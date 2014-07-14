#include "Stabilizer.h"

#include <Eigen/Dense>
#include <VirtualRobot/MathTools.h>

#include <boost/assert.hpp>
#include <boost/make_shared.hpp>

#include "ChestPostureController.h"
#include "FootForceController.h"
#include "FootTorqueController.h"
#include "ForceDistributor.h"

#include "../../Sensors/ForceSensor.h"
#include "../../IK/HierarchicalReferenceIK.h"
#include "../../Utils/CSVLogger.h"

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

Stabilizer::Stabilizer(SimDynamics::DynamicsRobotPtr robot,
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

    for (auto& name : motion->getJointNames())
    {
        trajectoryNodes.push_back(robot->getRobot()->getRobotNode(name));
    }
}

void Stabilizer::control(float dt)
{
    if (!currentFrame)
        return;

    Kinematics::SupportPhase phase;
    int value = 0;
    if(GetControlValue(currentFrame, "SupportPhase", value))
    {
        phase = static_cast<Kinematics::SupportPhase>(value);
    }

    Eigen::Vector2f zmp2D;
    if (!GetControlPointPosition(currentFrame, "ZMP", zmp2D))
        return;
    Eigen::Vector3f zmp = Eigen::Vector3f::Zero();
    zmp.head(2) = zmp2D;

    Eigen::Matrix4f chestPoseRef;
    if (!GetControlMatrix(currentFrame, "Chest", chestPoseRef))
        return;

    Eigen::Matrix4f pelvisPoseRef;
    if (!GetControlMatrix(currentFrame, "Pelvis", pelvisPoseRef))
        return;

    Eigen::Matrix4f leftFootPoseRef;
    if (!GetControlMatrix(currentFrame, "LeftFoot", leftFootPoseRef))
        return;

    Eigen::Matrix4f rightFootPoseRef;
    if (!GetControlMatrix(currentFrame, "RightFoot", rightFootPoseRef))
        return;

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


    /*
    for (unsigned i = 0; i < trajectoryNodes.size(); i++)
    {
        trajectoryNodes[i]->setJointValue(currentFrame->joint[i]);
    }*/

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

void Stabilizer::log(float dt)
{
    if (logger)
    {
        logger->log(dt, {
            chestPostureController->currentRPY.x(),
            chestPostureController->currentRPY.y(),
            chestPostureController->refRPY.x(),
            chestPostureController->refRPY.y(),
            chestPostureController->phiDC.delta,
            chestPostureController->thetaDC.delta
        });
    }
}

void Stabilizer::enableLogging(const std::string& path)
{
    std::vector<std::string> cols = {"ChestActualPhi", "ChestActualTheta",
                                     "ChestTargetPhi", "ChestTargetTheta",
                                     "ChestDeltaPhi", "ChestDeltaTheta"
                                     };
    logger = boost::make_shared<CSVLogger<double>>(path, cols);
}

