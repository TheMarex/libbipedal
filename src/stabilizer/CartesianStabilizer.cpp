#include <Eigen/Dense>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Nodes/ForceTorqueSensor.h>
#include <SimDynamics/SimDynamics.h>

#include <boost/assert.hpp>
#include <boost/make_shared.hpp>

#include "stabilizer/CartesianStabilizer.h"

#include "ik/DifferentialReferenceIK.h"

#include "controller/TwoDOFPostureController.h"

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
 * Points:
 *  - Reference ZMP
 */
CartesianStabilizer::CartesianStabilizer(const VirtualRobot::RobotPtr& robot,
                                         const VirtualRobot::RobotNodeSetPtr& nodes,
                                         const VirtualRobot::RobotNodePtr& chest,
                                         const VirtualRobot::RobotNodePtr& leftFoot,
                                         const VirtualRobot::RobotNodePtr& rightFoot,
                                         const VirtualRobot::RobotNodePtr& pelvis,
                                         ReferenceIKPtr referenceIK)
: chest(chest)
, leftFoot(leftFoot)
, rightFoot(rightFoot)
, pelvis(pelvis)
, chestPostureController(new TwoDOFPostureController(5, 50, 5, 50))
, leftFootPostureController(new TwoDOFPostureController(20, 10, 20, 10))
, rightFootPostureController(new TwoDOFPostureController(20, 10, 20, 10))
, pelvisPostureController(new TwoDOFPostureController(5, 20, 5, 20))
, nodes(nodes)
, referenceIK(referenceIK)
, chestPose(Eigen::Matrix4f::Identity())
, pelvisPose(Eigen::Matrix4f::Identity())
, leftFootPose(Eigen::Matrix4f::Identity())
, rightFootPose(Eigen::Matrix4f::Identity())
, rootPose(Eigen::Matrix4f::Identity())
, comPosition(Eigen::Vector3f::Zero())
, comPositionRef(Eigen::Vector3f::Zero())
, zmpPositionRef(Eigen::Vector3f::Zero())
, stepAdaptionFrame(Eigen::Matrix4f::Zero())
{
}

void CartesianStabilizer::update(float dt,
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

    // Reference coordinate system for orientations is the ground frame
    // not the global frame
    // Note: We don't care about positions here, only the orientation part is used
    Eigen::Matrix4f refToWorld = computeGroundFrame(leftFoot->getGlobalPose(), rightFoot->getGlobalPose(), phase);
    Eigen::Matrix4f worldToRef = refToWorld.inverse();

    pelvisPose = refToWorld * pelvisPostureController->correctPosture(
        worldToRef * pelvisPoseRef,
        worldToRef * pelvis->getGlobalPose()
    );

    chestPose = refToWorld * chestPostureController->correctPosture(
        worldToRef * chestPoseRef,
        worldToRef * chest->getGlobalPose()
    );

    leftFootPose = refToWorld * leftFootPostureController->correctPosture(
        worldToRef * leftFootPoseRef,
        worldToRef * leftFoot->getGlobalPose()
    );

    rightFootPose = refToWorld * rightFootPostureController->correctPosture(
        worldToRef * rightFootPoseRef,
        worldToRef * rightFoot->getGlobalPose()
    );

    //leftFootPose  = leftFootPoseRef;
    //rightFootPose = rightFootPoseRef;
    //chestPose     = chestPoseRef;
    //pelvisPose    = pelvisPoseRef;
    comPosition   = comPositionRef;

    std::vector<float> angles;
    auto originalRoot = nodes->getRobot()->getGlobalPose();
    nodes->getJointValues(angles);
    bool success = referenceIK->computeStep(leftFootPose, rightFootPose, chestPose, pelvisPose, comPosition, phase, resultAngles);
    nodes->setJointValues(angles);
    nodes->getRobot()->setGlobalPose(originalRoot);
    BOOST_ASSERT(!std::isnan(resultAngles[0]));
}

std::unordered_map<std::string, DampeningController*> CartesianStabilizer::getControllers()
{
    std::unordered_map<std::string, DampeningController*> controllers;

    controllers["Chest_Roll"]     = &chestPostureController->phiDC;
    controllers["Chest_Pitch"]    = &chestPostureController->thetaDC;
    controllers["LeftFoot_Roll"]  = &leftFootPostureController->phiDC;
    controllers["LeftFoot_Pitch"] = &leftFootPostureController->thetaDC;
    controllers["RightFoot_Roll"]  = &rightFootPostureController->phiDC;
    controllers["RightFoot_Pitch"] = &rightFootPostureController->thetaDC;
    controllers["PelvisFoot_Roll"]  = &pelvisPostureController->phiDC;
    controllers["PelvisFoot_Pitch"] = &pelvisPostureController->thetaDC;

    return controllers;
}
