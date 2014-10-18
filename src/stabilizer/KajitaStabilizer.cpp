#include <Eigen/Dense>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Nodes/ForceTorqueSensor.h>
#include <SimDynamics/SimDynamics.h>

#include <boost/assert.hpp>
#include <boost/make_shared.hpp>

#include <unordered_map>

#include "kajita/FootForceController.h"
#include "kajita/FootTorqueController.h"
#include "kajita/ZMPTrackingController.h"

#include "stabilizer/kajita/ZMPDistributor.h"
#include "stabilizer/KajitaStabilizer.h"

#include "controller/PostureController.h"

namespace Bipedal
{

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
                                   const VirtualRobot::RobotNodePtr& chest,
                                   const VirtualRobot::RobotNodePtr& leftFoot,
                                   const VirtualRobot::RobotNodePtr& rightFoot,
                                   const VirtualRobot::RobotNodePtr& leftFootBody,
                                   const VirtualRobot::RobotNodePtr& rightFootBody,
                                   const VirtualRobot::RobotNodePtr& leftAnkleBody,
                                   const VirtualRobot::RobotNodePtr& rightAnkleBody,
                                   const VirtualRobot::RobotNodePtr& pelvis,
                                   const VirtualRobot::ForceTorqueSensorPtr& leftAnkleSensorX,
                                   const VirtualRobot::ForceTorqueSensorPtr& rightAnkleSensorX,
                                   const VirtualRobot::ForceTorqueSensorPtr& leftAnkleSensorY,
                                   const VirtualRobot::ForceTorqueSensorPtr& rightAnkleSensorY)
/* Nodes */
: chest(chest)
, leftFoot(leftFoot)
, rightFoot(rightFoot)
, leftFootBody(leftFootBody)
, rightFootBody(rightFootBody)
, pelvis(pelvis)
, leftAnkleSensorX(leftAnkleSensorX)
, rightAnkleSensorX(rightAnkleSensorX)
, leftAnkleSensorY(leftAnkleSensorY)
, rightAnkleSensorY(rightAnkleSensorY)
/* Controllers */
, chestPostureController(new TwoDOFPostureController(40, 5, 80, 5))
, forceDistributor(new ZMPDistributor(robot->getMass(),
                                        Eigen::Vector3f(0.0, 0.0, -9.81),
                                        leftFootBody, rightFootBody, leftFoot, rightFoot))
, footTorqueController(new FootTorqueController())
// gains calculated with utils/zmp_gains.ipynote
, zmpTrackingController(new ZMPTrackingController(Eigen::Vector3f(-0.09150757, -0.02709391, -0.33112892)))
/* Adapted frames */
, chestPose(Eigen::Matrix4f::Identity())
, pelvisPose(Eigen::Matrix4f::Identity())
, leftFootPose(Eigen::Matrix4f::Identity())
, rightFootPose(Eigen::Matrix4f::Identity())
, zmpPosition(Eigen::Vector3f::Zero())
, comPosition(Eigen::Vector3f::Zero())
, comPositionRef(Eigen::Vector3f::Zero())
, zmpPositionRef(Eigen::Vector3f::Zero())
, stepAdaptionFrame(Eigen::Matrix4f::Zero())
, ft({Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()})
{
    BOOST_ASSERT(robot);
    BOOST_ASSERT(chest);
    BOOST_ASSERT(leftFoot);
    BOOST_ASSERT(rightFoot);
    BOOST_ASSERT(leftFootBody);
    BOOST_ASSERT(rightFootBody);
    BOOST_ASSERT(leftAnkleBody);
    BOOST_ASSERT(rightAnkleBody);
    BOOST_ASSERT(pelvis);
    BOOST_ASSERT(leftAnkleSensorX);
    BOOST_ASSERT(rightAnkleSensorX);
    BOOST_ASSERT(leftAnkleSensorY);
    BOOST_ASSERT(rightAnkleSensorY);

    // FIXME Make node name configurable
    Eigen::Vector3f leftHipPos  = robot->getRobotNode("LeftLeg_Joint2")->getGlobalPose().block(0, 3, 3, 1);
    Eigen::Vector3f rightHipPos = robot->getRobotNode("RightLeg_Joint2")->getGlobalPose().block(0, 3, 3, 1);
    double hipJointDistance = (leftHipPos - rightHipPos).norm();
    footForceController = FootForceControllerPtr(new FootForceController(hipJointDistance));

    leftAnkleOffset = (leftFoot->getGlobalPose().inverse() * leftAnkleBody->getGlobalPose()).block(0,3,3,1);
    // project to foot sole
    leftAnkleOffset.z() = 0;
    rightAnkleOffset = (rightFoot->getGlobalPose().inverse() * rightAnkleBody->getGlobalPose()).block(0,3,3,1);
    // project to foot sole
    rightAnkleOffset.z() = 0;
}

void KajitaStabilizer::update(float dt,
                              const Eigen::Vector3f& comActualWorld,
                              const Eigen::Vector3f& comVelocityActualWorld,
                              const Eigen::Vector3f& zmpActualWorld,
                              Bipedal::SupportPhase phase,
                              const Eigen::Matrix4f& chestPoseRefWorld,
                              const Eigen::Matrix4f& pelvisPoseRefWorld,
                              const Eigen::Matrix4f& leftFootPoseRefWorld,
                              const Eigen::Matrix4f& rightFootPoseRefWorld,
                              const Eigen::Vector3f& comRefGroundFrame,
                              const Eigen::Vector3f& comVelocityRefGroundFrame,
                              const Eigen::Vector3f& zmpRefGroundFrame)
{
    Eigen::Matrix4f leftToWorld  = leftFoot->getGlobalPose();
    Eigen::Matrix4f rightToWorld = rightFoot->getGlobalPose();
    auto groundFrame = Bipedal::computeGroundFrame(leftToWorld, rightToWorld, phase);
    auto worldToGroundFrame = groundFrame.inverse();

    // orient reference trajectory to initial orientation of robot
    if (stepAdaptionFrame == Eigen::Matrix4f::Zero())
    {
        std::cout << "Initializing step adaption frame..." << std::endl;
        stepAdaptionFrame = groundFrame
          * computeGroundFrame(leftFootPoseRefWorld, rightFootPoseRefWorld, phase).inverse();
        std::cout << "Step adaption frame: " << stepAdaptionFrame << std::endl;
    }

    // Apply step adation frame to reference values
    chestPoseRef     = stepAdaptionFrame * chestPoseRefWorld;
    pelvisPoseRef    = stepAdaptionFrame * pelvisPoseRefWorld;
    leftFootPoseRef  = stepAdaptionFrame * leftFootPoseRefWorld;
    rightFootPoseRef = stepAdaptionFrame * rightFootPoseRefWorld;
    comPositionRef   = comRefGroundFrame;
    zmpPositionRef   = zmpRefGroundFrame;

    // We never change the reference
    comPosition = comPositionRef;

    // convert actual values to ground frame
    Eigen::Vector3f zmpActualGroundFrame = VirtualRobot::MathTools::transformPosition(zmpActualWorld, worldToGroundFrame);
    Eigen::Vector3f comActualGroundFrame = VirtualRobot::MathTools::transformPosition(comActualWorld, worldToGroundFrame);
    Eigen::Vector3f comVelocityActualGroundFrame = worldToGroundFrame.block(0,0,3,3) * comVelocityActualWorld;

    // Adapt the reference ZMP to stabilize
    zmpPosition = Eigen::Vector3f::Zero();
    zmpPosition.head(2) = zmpTrackingController->computeAdaptedZMP(zmpRefGroundFrame.head(2),
                                                                   comRefGroundFrame.head(2),
                                                                   comVelocityRefGroundFrame.head(2),
                                                                   zmpActualGroundFrame.head(2),
                                                                   comActualGroundFrame.head(2),
                                                                   comVelocityActualGroundFrame.head(2));

    // Distribute adapted ZMP to yielf reference force/torques
    ft = forceDistributor->distributeZMP(
        leftAnkleOffset,
        rightAnkleOffset,
        leftToWorld,
        rightToWorld,
        zmpPosition,
        phase
    );

    Eigen::Matrix3f worldToLeft  = leftToWorld.block(0, 0, 3, 3).inverse();
    Eigen::Matrix3f worldToRight = rightToWorld.block(0, 0, 3, 3).inverse();
    Eigen::Vector3f leftTorqueLocal  = worldToLeft * (leftAnkleSensorX->getAxisTorque()  + leftAnkleSensorY->getAxisTorque());
    Eigen::Vector3f rightTorqueLocal = worldToRight * (rightAnkleSensorX->getAxisTorque() + rightAnkleSensorY->getAxisTorque());

    leftFootPose  = Eigen::Matrix4f::Identity();
    rightFootPose = Eigen::Matrix4f::Identity();
    footTorqueController->correctFootOrientation(
        leftFootPoseRef,
        rightFootPoseRef,
        ft.leftTorque,
        ft.rightTorque,
        leftTorqueLocal,
        rightTorqueLocal,
        leftFootPose,
        rightFootPose
    );

/* FIXME RobotConfig needs a flag for deciding what to use
    pelvisPose = footForceController->correctPelvisOrientation(
        pelvisPoseRef,
        ft.leftForce,
        ft.rightForce,
        -leftAnkleSensorX->getForce(),
        rightAnkleSensorX->getForce()
    );
*/
    pelvisPose = pelvisPoseRef;
    footForceController->correctFootHeight(
        ft.leftForce,
        ft.rightForce,
        -leftAnkleSensorX->getForce(),
        rightAnkleSensorX->getForce(),
        leftFootPose,
        rightFootPose
    );

    chestPose = chestPoseRef * chestPostureController->correctPosture(
        chestPoseRef,
        chest->getGlobalPose()
    );
}

std::unordered_map<std::string, DampeningController*> KajitaStabilizer::getControllers()
{
    std::unordered_map<std::string, DampeningController*> controllers;

    controllers["LeftAnkle_TorqueX"]  = &footTorqueController->leftPhiDC;
    controllers["LeftAnkle_TorqueY"]  = &footTorqueController->leftThetaDC;
    controllers["RightAnkle_TorqueX"] = &footTorqueController->rightPhiDC;
    controllers["RightAnkle_TorqueY"] = &footTorqueController->rightThetaDC;
    controllers["Chest_Roll"]         = &chestPostureController->phiDC;
    controllers["Chest_Pitch"]        = &chestPostureController->thetaDC;
    controllers["Pelvis_Pitch"]       = &footForceController->zCtrlDC;

    return controllers;
}

}

