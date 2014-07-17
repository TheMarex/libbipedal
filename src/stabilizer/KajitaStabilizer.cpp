#include <Eigen/Dense>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Nodes/ForceTorqueSensor.h>
#include <SimDynamics/SimDynamics.h>

#include <boost/assert.hpp>
#include <boost/make_shared.hpp>

#include "kajita/ChestPostureController.h"
#include "kajita/FootForceController.h"
#include "kajita/FootTorqueController.h"
#include "kajita/ForceDistributor.h"

#include "stabilizer/KajitaStabilizer.h"

#include "ik/HierarchicalReferenceIK.h"

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
, nodes(robot->getRobotNodeSet("Robot"))
, referenceIK(boost::dynamic_pointer_cast<ReferenceIK>(
                boost::make_shared<HierarchicalReferenceIK>(nodes, robot, leftFoot, rightFoot, chest, pelvis)
              ))
, chestPose(Eigen::Matrix4f::Identity())
, pelvisPose(Eigen::Matrix4f::Identity())
, leftFootPose(Eigen::Matrix4f::Identity())
, rightFootPose(Eigen::Matrix4f::Identity())
, rootPose(Eigen::Matrix4f::Identity())
{
    Eigen::Vector3f leftHipPos  = robot->getRobotNode("LeftLeg_Joint2")->getGlobalPose().block(0, 3, 3, 1);
    Eigen::Vector3f rightHipPos = robot->getRobotNode("RightLeg_Joint2")->getGlobalPose().block(0, 3, 3, 1);
    double hipJointDistance = (leftHipPos - rightHipPos).norm();
    footForceController = FootForceControllerPtr(new FootForceController(hipJointDistance));

}

void KajitaStabilizer::update(float dt,
                              Kinematics::SupportPhase phase,
                              const Eigen::Vector3f& zmp,
                              const Eigen::Matrix4f& chestPoseRefWorld,
                              const Eigen::Matrix4f& pelvisPoseRefWorld,
                              const Eigen::Matrix4f& leftFootPoseRefWorld,
                              const Eigen::Matrix4f& rightFootPoseRefWorld)
{

    Eigen::Matrix4f stepAdaptionFrame =
        computeGroundFrame(leftFoot->getGlobalPose(), rightFoot->getGlobalPose(), phase)
      * computeGroundFrame(leftFootPoseRefWorld, rightFootPoseRefWorld, phase).inverse();

    Eigen::Matrix4f chestPoseRef = stepAdaptionFrame * chestPoseRefWorld;
    Eigen::Matrix4f pelvisPoseRef = stepAdaptionFrame * pelvisPoseRefWorld;
    Eigen::Matrix4f leftFootPoseRef = stepAdaptionFrame * leftFootPoseRefWorld;
    Eigen::Matrix4f rightFootPoseRef = stepAdaptionFrame * rightFootPoseRefWorld;

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

    bool success = referenceIK->computeStep(leftFootPose, rightFootPose, chestPose, pelvisPose, phase, resultAngles);
    BOOST_ASSERT(!std::isnan(resultAngles[0]));
}

