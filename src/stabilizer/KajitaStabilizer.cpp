#include <Eigen/Dense>
#include <VirtualRobot/MathTools.h>

#include <boost/assert.hpp>
#include <boost/make_shared.hpp>

#include "kajita/ChestPostureController.h"
#include "kajita/FootForceController.h"
#include "kajita/FootTorqueController.h"
#include "kajita/ForceDistributor.h"

#include "stabilizer/KajitaStabilizer.h"

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
KajitaStabilizer::KajitaStabilizer(SimDynamics::DynamicsRobotPtr robot,
               const ForceSensorPtr& leftAnkleSensor,
               const ForceSensorPtr& rightAnkleSensor,
               const std::string& motionPath,
               const std::string& goalMotionName)
: TrajectoryController(robot, motionPath, goalMotionName)
// Names specific to ARMAR 4
, chest(robot->getRobot()->getRobotNode("TorsoCenter"))
, leftFoot(robot->getRobot()->getRobotNode("LeftLeg_TCP"))
, rightFoot(robot->getRobot()->getRobotNode("RightLeg_TCP"))
, leftAnkle(robot->getRobot()->getRobotNode("LeftLeg_BodyAnkle2"))
, rightAnkle(robot->getRobot()->getRobotNode("RightLeg_BodyAnkle2"))
, pelvis(robot->getRobot()->getRobotNode("Waist"))
, leftAnkleSensor(leftAnkleSensor)
, rightAnkleSensor(rightAnkleSensor)
, chestPostureController(new ChestPostureController())
, forceDistributor(new ForceDistributor(robot->getRobot()->getMass(),
                                        Eigen::Vector3f(0.0, 0.0, -9.81),
                                        leftAnkle, rightAnkle, leftFoot, rightFoot))
, footTorqueController(new FootTorqueController())
, nodes(robot->getRobot()->getRobotNodeSet("Robot"))
, referenceIK(boost::dynamic_pointer_cast<ReferenceIK>(
                boost::make_shared<HierarchicalReferenceIK>(nodes, robot->getRobot(), leftFoot, rightFoot, chest, pelvis)
              ))
, chestPose(Eigen::Matrix4f::Identity())
, pelvisPose(Eigen::Matrix4f::Identity())
, leftFootPose(Eigen::Matrix4f::Identity())
, rightFootPose(Eigen::Matrix4f::Identity())
, rootPose(Eigen::Matrix4f::Identity())
{
    Eigen::Vector3f leftHipPos  = robot->getRobot()->getRobotNode("LeftLeg_Joint2")->getGlobalPose().block(0, 3, 3, 1);
    Eigen::Vector3f rightHipPos = robot->getRobot()->getRobotNode("RightLeg_Joint2")->getGlobalPose().block(0, 3, 3, 1);
    double hipJointDistance = (leftHipPos - rightHipPos).norm();
    footForceController = FootForceControllerPtr(new FootForceController(hipJointDistance));

}

void KajitaStabilizer::update(float dt,
                              Kinematics::SupportPhase phase,
                              const Eigen::Vector3f& zmp,
                              const Eigen::Matrix4f& chestPoseRef,
                              const Eigen::Matrix4f& pelvisPoseRef,
                              const Eigen::Matrix4f& leftFootPoseRef,
                              const Eigen::Matrix4f& rightFootPoseRef)
{

    Eigen::Matrix4f stepAdaptionFrame =
        computeGroundFrame(leftFoot->getGlobalPose(), rightFoot->getGlobalPose(), phase)
      * computeGroundFrame(leftFootPoseRef, rightFootPoseRef, phase).inverse();

    chestPoseRef = stepAdaptionFrame * chestPoseRef;
    pelvisPoseRef = stepAdaptionFrame * pelvisPoseRef;
    leftFootPoseRef = stepAdaptionFrame * leftFootPoseRef;
    rightFootPoseRef = stepAdaptionFrame * rightFootPoseRef;

    ForceDistributor::ForceTorque ft = forceDistributor->distributeZMP(
        leftFoot->getGlobalPose(),
        rightFoot->getGlobalPose(),
        zmp,
        phase
    );

    leftFootPose  = Eigen::Matrix4f::Identity();
    rightFootPose = Eigen::Matrix4f::Identity();
    footTorqueController->correctFootOrientation(
        leftFootPoseRef,
        rightFootPoseRef,
        ft.leftTorque,
        ft.rightTorque,
        leftAnkleSensor->getTorque(),
        rightAnkleSensor->getTorque(),
        leftFootPose,
        rightFootPose
    );

    pelvisPose = footForceController->correctPelvisOrientation(
        pelvisPoseRef,
        ft.leftForce,
        ft.rightForce,
        leftAnkleSensor->getForce(),
        rightAnkleSensor->getForce()
    );

    chestPose = chestPostureController->correctPosture(
        chestPoseRef,
        chest->getGlobalPose()
    );

    leftFootPose = leftFootPoseRef;
    rightFootPose = rightFootPoseRef;
    chestPose = chestPoseRef;
    pelvisPose = pelvisPoseRef;


    rootPose = robot->getRobot()->getRootNode()->getGlobalPose();
    bool success = referenceIK->computeStep(leftFootPose, rightFootPose, chestPose, pelvisPose, phase, resultAngles);

    BOOST_ASSERT(!std::isnan(resultAngles[0]));

//    if (success)
//    {
        for (unsigned i = 0; i < nodes->getSize(); i++)
        {
            robot->actuateNode((*nodes)[i], resultAngles(i, 0));
        }
//    }
}

