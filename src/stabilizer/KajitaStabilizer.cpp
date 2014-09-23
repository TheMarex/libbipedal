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

#include "stabilizer/kajita/ForceDistributor.h"
#include "stabilizer/KajitaStabilizer.h"

#include "ik/DifferentialReferenceIK.h"

#include "controller/PostureController.h"

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
                                   const VirtualRobot::RobotNodeSetPtr& nodes,
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
                                   const VirtualRobot::ForceTorqueSensorPtr& rightAnkleSensorY,
                                   ReferenceIKPtr referenceIK)
/* Nodes */
: chest(chest)
, leftFoot(leftFoot)
, rightFoot(rightFoot)
, leftFootBody(leftFootBody)
, rightFootBody(rightFootBody)
, leftAnkleBody(leftAnkleBody)
, rightAnkleBody(rightAnkleBody)
, pelvis(pelvis)
, leftAnkleSensorX(leftAnkleSensorX)
, rightAnkleSensorX(rightAnkleSensorX)
, leftAnkleSensorY(leftAnkleSensorY)
, rightAnkleSensorY(rightAnkleSensorY)
, nodes(nodes)
/* Controllers */
, chestPostureController(new TwoDOFPostureController(40, 5, 80, 5))
, forceDistributor(new ForceDistributor(robot->getMass(),
                                        Eigen::Vector3f(0.0, 0.0, -9.81),
                                        leftFootBody, rightFootBody, leftFoot, rightFoot))
, footTorqueController(new FootTorqueController())
// gains from paper
, zmpTrackingController(new ZMPTrackingController(Eigen::Vector3f(-13, -3, -3.377)))
/* Adapted frames */
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
, referenceIK(referenceIK)
, refCoMVelEstimator(Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero())
{
    // FIXME Make node name configurable
    Eigen::Vector3f leftHipPos  = robot->getRobotNode("LeftLeg_Joint2")->getGlobalPose().block(0, 3, 3, 1);
    Eigen::Vector3f rightHipPos = robot->getRobotNode("RightLeg_Joint2")->getGlobalPose().block(0, 3, 3, 1);
    double hipJointDistance = (leftHipPos - rightHipPos).norm();
    footForceController = FootForceControllerPtr(new FootForceController(hipJointDistance));

    BOOST_ASSERT(robot);
    BOOST_ASSERT(nodes);
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
    BOOST_ASSERT(referenceIK);

/*
    std::cout
     << "KajitaStabilizer nodes:" << std::endl
     << "robot: " << robot->getName() << std::endl
     << "nodes: " << nodes->getName() << std::endl
     << "chest: " << chest->getName() << std::endl
     << "leftFoot: " << leftFoot->getName() << std::endl
     << "rightFoot: " << rightFoot->getName() << std::endl
     << "leftFootBody: " << leftFootBody->getName() << std::endl
     << "rightFootBody: " << rightFootBody->getName() << std::endl
     << "leftAnkleBody: " << leftAnkleBody->getName() << std::endl
     << "rightAnkleBody: " << rightAnkleBody->getName() << std::endl
     << "pelvis: " << pelvis->getName() << std::endl
     << "leftAnkleSensorX: " << leftAnkleSensorX->getName() << std::endl
     << "rightAnkleSensorX: " << rightAnkleSensorX->getName() << std::endl
     << "leftAnkleSensorY: " << leftAnkleSensorY->getName() << std::endl
     << "rightAnkleSensorY: " << rightAnkleSensorY->getName() << std::endl
     << std::endl;
*/
}

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

    refCoMVelEstimator.update(comPositionRef.head(2), dt);
    //zmpTrackingController->adaptReferenceZMP(zmpPositionRef.head(2), comPositionRef.head(2), refCoMVelEstimator.estimation,);

    Eigen::Matrix4f leftToWorld  = leftFoot->getGlobalPose();
    Eigen::Matrix4f rightToWorld = rightFoot->getGlobalPose();
    ft = forceDistributor->distributeZMP(
        leftAnkleBody->getGlobalPose().block(0, 3, 3, 1),
        rightAnkleBody->getGlobalPose().block(0, 3, 3, 1),
        leftToWorld,
        rightToWorld,
        zmpPositionRef,
        phase
    );

    Eigen::Matrix3f worldToLeft  = leftToWorld.block(0, 0, 3, 3).inverse();
    Eigen::Matrix3f worldToRight = rightToWorld.block(0, 0, 3, 3).inverse();
    Eigen::Vector3f leftTorqueWorld  = leftAnkleSensorX->getAxisTorque()  + leftAnkleSensorY->getAxisTorque();
    Eigen::Vector3f rightTorqueWorld = rightAnkleSensorX->getAxisTorque() + rightAnkleSensorY->getAxisTorque();

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
        -leftAnkleSensorX->getForce(),
        rightAnkleSensorX->getForce()
    );

    chestPose = chestPoseRef * chestPostureController->correctPosture(
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

