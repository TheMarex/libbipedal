#include <Eigen/Dense>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Nodes/ForceTorqueSensor.h>
#include <SimDynamics/SimDynamics.h>

#include <boost/assert.hpp>
#include <boost/make_shared.hpp>

#include "stabilizer/CartesianStabilizer.h"
#include "utils/Kinematics.h"

#include "controller/PostureController.h"

namespace Bipedal
{

/**
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
 */
CartesianStabilizer::CartesianStabilizer(const VirtualRobot::RobotPtr& robot,
                                         const VirtualRobot::RobotNodePtr& chest,
                                         const VirtualRobot::RobotNodePtr& leftFoot,
                                         const VirtualRobot::RobotNodePtr& rightFoot,
                                         const VirtualRobot::RobotNodePtr& pelvis)
: chest(chest)
, leftFoot(leftFoot)
, rightFoot(rightFoot)
, pelvis(pelvis)
, chestPostureController(new ThreeDOFPostureController(8, 12, 8, 12, 8, 12))
, leftFootPostureController(new ThreeDOFPostureController(10, 17, 15, 30, 20, 20))
, rightFootPostureController(new ThreeDOFPostureController(10, 17, 15, 30, 20, 20))
, pelvisPostureController(new ThreeDOFPostureController(8, 12, 8, 12, 8, 12))
, chestPose(Eigen::Matrix4f::Identity())
, pelvisPose(Eigen::Matrix4f::Identity())
, leftFootPose(Eigen::Matrix4f::Identity())
, rightFootPose(Eigen::Matrix4f::Identity())
, chestPoseRef(Eigen::Matrix4f::Identity())
, pelvisPoseRef(Eigen::Matrix4f::Identity())
, leftFootPoseRef(Eigen::Matrix4f::Identity())
, rightFootPoseRef(Eigen::Matrix4f::Identity())
, rootPose(Eigen::Matrix4f::Identity())
, comPosition(Eigen::Vector3f::Zero())
, zmpPosition(Eigen::Vector3f::Zero())
, comPositionRef(Eigen::Vector3f::Zero())
, zmpPositionRef(Eigen::Vector3f::Zero())
, stepAdaptionFrame(Eigen::Matrix4f::Zero())
{
}

void CartesianStabilizer::update(float dt,
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

    // Reference coordinate system for orientations is the ground frame
    // not the global frame
    // Note: We don't care about positions here, only the orientation part is used
    Eigen::Matrix4f worldToRefGF = computeGroundFrame(leftFootPoseRef, rightFootPoseRef, phase).inverse();
    Eigen::Matrix4f worldToGF = computeGroundFrame(leftFoot->getGlobalPose(), rightFoot->getGlobalPose(), phase).inverse();

    pelvisPose = pelvisPoseRef * pelvisPostureController->correctPosture(
        worldToRefGF * pelvisPoseRef,
        worldToGF * pelvis->getGlobalPose()
    );

    chestPose = chestPoseRef * chestPostureController->correctPosture(
        worldToRefGF * chestPoseRef,
        worldToGF * chest->getGlobalPose()
    );

    Eigen::Matrix4f leftCorrection = leftFootPostureController->correctPosture(
        worldToRefGF * leftFootPoseRef,
        worldToGF * leftFoot->getGlobalPose()
    );
    Eigen::Matrix4f rightCorrection = rightFootPostureController->correctPosture(
        worldToRefGF * rightFootPoseRef,
        worldToGF * rightFoot->getGlobalPose()
    );

    leftFootPose = leftFootPoseRef * leftCorrection;
    rightFootPose = rightFootPoseRef * rightCorrection;

    comPositionRef = comRefGroundFrame;
    zmpPositionRef = zmpRefGroundFrame;
    comPosition   = comPositionRef;
    zmpPosition   = zmpPositionRef;
}

std::unordered_map<std::string, DampeningController*> CartesianStabilizer::getControllers()
{
    std::unordered_map<std::string, DampeningController*> controllers;

    controllers["Chest_Roll"]     = &chestPostureController->phiDC;
    controllers["Chest_Pitch"]    = &chestPostureController->thetaDC;
    controllers["Chest_Yaw"]    = &chestPostureController->gammaDC;
    controllers["LeftFoot_Roll"]  = &leftFootPostureController->phiDC;
    controllers["LeftFoot_Pitch"] = &leftFootPostureController->thetaDC;
    controllers["LeftFoot_Yaw"] = &leftFootPostureController->gammaDC;
    controllers["RightFoot_Roll"]  = &rightFootPostureController->phiDC;
    controllers["RightFoot_Pitch"] = &rightFootPostureController->thetaDC;
    controllers["RightFoot_Yaw"] = &rightFootPostureController->gammaDC;
    controllers["Pelvis_Roll"]  = &pelvisPostureController->phiDC;
    controllers["Pelvis_Pitch"] = &pelvisPostureController->thetaDC;
    controllers["Pelvis_Yaw"] = &pelvisPostureController->gammaDC;

    return controllers;
}

}
