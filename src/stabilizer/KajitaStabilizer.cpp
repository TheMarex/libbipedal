#include <Eigen/Dense>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Nodes/ForceTorqueSensor.h>
#include <SimDynamics/SimDynamics.h>

#include <boost/assert.hpp>
#include <boost/make_shared.hpp>

#include "kajita/ChestPostureController.h"
#include "kajita/FootForceController.h"
#include "kajita/FootTorqueController.h"

#include "stabilizer/kajita/ForceDistributor.h"
#include "stabilizer/KajitaStabilizer.h"

#include "ik/DifferentialReferenceIK.h"

/*
 * These controller needs a trajectory with the following control features:
 *
 * Poses:
 *  - chest pose
 *  - pelvis pose
 *  - left foot pose
 *  - right foot pose
 *
 * Values:
 *  - support phase
 *
 * Points:
 *  - Reference ZMP
 */
KajitaStabilizer::KajitaStabilizer(const VirtualRobot::RobotPtr& robot,
               const VirtualRobot::ForceTorqueSensorPtr& leftAnkleSensor,
               const VirtualRobot::ForceTorqueSensorPtr& rightAnkleSensor)
// Names specific to ARMAR 4
: chest(robot->getRobotNode("TorsoCenter"))
, leftFoot(robot->getRobotNode("LeftLeg_TCP"))
, rightFoot(robot->getRobotNode("RightLeg_TCP"))
, leftAnkle(robot->getRobotNode("LeftLeg_BodyAnkle2"))
, rightAnkle(robot->getRobotNode("RightLeg_BodyAnkle2"))
, pelvis(robot->getRobotNode("Waist"))
, leftAnkleSensor(leftAnkleSensor)
, rightAnkleSensor(rightAnkleSensor)
, chestPostureController(new ChestPostureController())
, forceDistributor(new ForceDistributor(robot->getMass(),
                                        Eigen::Vector3f(0.0, 0.0, -9.81),
                                        leftAnkle, rightAnkle, leftFoot, rightFoot))
, footTorqueController(new FootTorqueController())
, nodes(robot->getRobotNodeSet("CoMCompensation"))
, referenceIK(boost::dynamic_pointer_cast<ReferenceIK>(
                boost::make_shared<DifferentialReferenceIK>(nodes, robot, robot->getRobotNodeSet("ColModelAll"), leftFoot, rightFoot, chest, pelvis)
              ))
, chestPose(Eigen::Matrix4f::Identity())
, pelvisPose(Eigen::Matrix4f::Identity())
, leftFootPose(Eigen::Matrix4f::Identity())
, rightFootPose(Eigen::Matrix4f::Identity())
, rootPose(Eigen::Matrix4f::Identity())
, comPosition(Eigen::Vector3f::Zero())
, comPositionRef(Eigen::Vector3f::Zero())
, zmpPositionRef(Eigen::Vector3f::Zero())
, stepAdaptionFrame(Eigen::Matrix4f::Zero())
, ft({Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()})
{
    Eigen::Vector3f leftHipPos  = robot->getRobotNode("LeftLeg_Joint2")->getGlobalPose().block(0, 3, 3, 1);
    Eigen::Vector3f rightHipPos = robot->getRobotNode("RightLeg_Joint2")->getGlobalPose().block(0, 3, 3, 1);
    double hipJointDistance = (leftHipPos - rightHipPos).norm();
    footForceController = FootForceControllerPtr(new FootForceController(hipJointDistance));

}

const DampeningController& KajitaStabilizer::getLeftAnkleTorqueXController() { return footTorqueController->leftPhiDC;}
const DampeningController& KajitaStabilizer::getLeftAnkleTorqueYController() { return footTorqueController->leftThetaDC;}
const DampeningController& KajitaStabilizer::getRightAnkleTorqueXController() { return footTorqueController->rightPhiDC;}
const DampeningController& KajitaStabilizer::getRightAnkleTorqueYController() { return footTorqueController->rightThetaDC;}
const DampeningController& KajitaStabilizer::getPelvisController() { return footForceController->zCtrlDC;}

void KajitaStabilizer::update(float dt,
                              Kinematics::SupportPhase phase,
                              const Eigen::Vector3f& zmpRefWorld,
                              const Eigen::Matrix4f& chestPoseRefWorld,
                              const Eigen::Matrix4f& pelvisPoseRefWorld,
                              const Eigen::Matrix4f& leftFootPoseRefWorld,
                              const Eigen::Matrix4f& rightFootPoseRefWorld,
                              const Eigen::Vector3f& comRefWorld)
{
    if (stepAdaptionFrame == Eigen::Matrix4f::Zero())
    {
        std::cout << "Initializing step adaption frame..." << std::endl;
        stepAdaptionFrame =
            computeGroundFrame(leftFoot->getGlobalPose(), rightFoot->getGlobalPose(), phase)
          * computeGroundFrame(leftFootPoseRefWorld, rightFootPoseRefWorld, phase).inverse();
        std::cout << "Step adaption frame: " << stepAdaptionFrame << std::endl;
    }

    chestPoseRef     = stepAdaptionFrame * chestPoseRefWorld;
    pelvisPoseRef    = stepAdaptionFrame * pelvisPoseRefWorld;
    leftFootPoseRef  = stepAdaptionFrame * leftFootPoseRefWorld;
    rightFootPoseRef = stepAdaptionFrame * rightFootPoseRefWorld;
    zmpPositionRef   = VirtualRobot::MathTools::transformPosition(zmpRefWorld, stepAdaptionFrame);
    comPositionRef   = VirtualRobot::MathTools::transformPosition(comRefWorld, stepAdaptionFrame);
    //chestPoseRef     = chestPoseRefWorld;
    //pelvisPoseRef    = pelvisPoseRefWorld;
    //leftFootPoseRef  = leftFootPoseRefWorld;
    //rightFootPoseRef = rightFootPoseRefWorld;

    Eigen::Matrix4f leftToWorld  = leftFoot->getGlobalPose();
    Eigen::Matrix4f rightToWorld = rightFoot->getGlobalPose();
    ft = forceDistributor->distributeZMP(
        leftToWorld,
        rightToWorld,
        zmpPositionRef,
        phase
    );

    Eigen::Matrix3f worldToLeft  = leftToWorld.block(0, 0, 3, 3).inverse();
    Eigen::Matrix3f worldToRight = rightToWorld.block(0, 0, 3, 3).inverse();
    Eigen::Vector3f leftTorqueWorld  = leftAnkleSensor->getTorque();
    Eigen::Vector3f rightTorqueWorld = rightAnkleSensor->getTorque();

    leftFootPose  = Eigen::Matrix4f::Identity();
    rightFootPose = Eigen::Matrix4f::Identity();
    footTorqueController->correctFootOrientation(
        leftFootPoseRef,
        rightFootPoseRef,
        ft.leftTorque,
        ft.rightTorque,
        worldToLeft * leftTorqueWorld,
        worldToRight * rightTorqueWorld,
        leftFootPose,
        rightFootPose
    );

    pelvisPose = footForceController->correctPelvisOrientation(
        pelvisPoseRef,
        ft.leftForce,
        ft.rightForce,
        -leftAnkleSensor->getForce(),
        rightAnkleSensor->getForce()
    );

    chestPose = chestPostureController->correctPosture(
        chestPoseRef,
        chest->getGlobalPose()
    );

/*
    leftFootPose  = leftFootPoseRef;
    rightFootPose = rightFootPoseRef;
    chestPose     = chestPoseRef;
    pelvisPose    = pelvisPoseRef;
*/
    comPosition   = comPositionRef;

    std::vector<float> angles;
    auto originalRoot = nodes->getRobot()->getGlobalPose();
    nodes->getJointValues(angles);
    bool success = referenceIK->computeStep(leftFootPose, rightFootPose, chestPose, pelvisPose, comPosition, phase, resultAngles);
    nodes->setJointValues(angles);
    nodes->getRobot()->setGlobalPose(originalRoot);
    BOOST_ASSERT(!std::isnan(resultAngles[0]));
}

VirtualRobot::RobotPtr KajitaStabilizer::getInvertedRobot()
{
    return boost::dynamic_pointer_cast<DifferentialReferenceIK>(referenceIK)->getInvertedRobot();
}
