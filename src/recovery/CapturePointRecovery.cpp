#include "bipedal.h"
#include "recovery/CapturePointRecovery.h"

#include "utils/Kinematics.h"
#include "utils/Estimation.h"
#include "utils/Walking.h"
#include "utils/Interpolation.h"
#include "utils/CapturePoint.h"

#include "controller/PostureController.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/MathTools.h>

#include <boost/make_shared.hpp>

namespace Bipedal
{

CapturePointRecovery::CapturePointRecovery(const VirtualRobot::RobotNodePtr& leftFoot,
                                           const VirtualRobot::RobotNodePtr& rightFoot,
                                           const VirtualRobot::RobotNodePtr& chest,
                                           const VirtualRobot::RobotNodePtr& pelvis)
: leftFoot(leftFoot)
, rightFoot(rightFoot)
, chest(chest)
, pelvis(pelvis)
, minHeight(100)
, minTime(0.35)
, gravity(9.81)
, recovering(false)
, leftFootPose(Eigen::Matrix4f::Identity())
, rightFootPose(Eigen::Matrix4f::Identity())
, chestPose(Eigen::Matrix4f::Identity())
, pelvisPose(Eigen::Matrix4f::Identity())
, initialLeftFootPose(Eigen::Matrix4f::Identity())
, initialRightFootPose(Eigen::Matrix4f::Identity())
, initialChestPose(Eigen::Matrix4f::Identity())
, initialPelvisPose(Eigen::Matrix4f::Identity())
, leftFootPostureController(new ThreeDOFPostureController(10, 17, 15, 30, 20, 20))
, rightFootPostureController(new ThreeDOFPostureController(10, 17, 15, 30, 20, 20))
{
}

const Eigen::Matrix4f& CapturePointRecovery::getLeftFootPose() const { return leftFootPose; }
const Eigen::Matrix4f& CapturePointRecovery::getRightFootPose() const { return rightFootPose; }
const Eigen::Matrix4f& CapturePointRecovery::getChestPose() const { return chestPose; }
const Eigen::Matrix4f& CapturePointRecovery::getPelvisPose() const { return pelvisPose; }
Bipedal::SupportPhase CapturePointRecovery::getSupportPhase() const { return recoverySupportPhase; }
const Bipedal::CubicBezierCurve3f& CapturePointRecovery::getCaptureTrajectory() const { return recoveryTrajectory; }

void CapturePointRecovery::update(const Eigen::Vector3f& com, const Eigen::Vector3f& comVel, double dt)
{
    comPosition = com;
    comVelocity = comVel;

    if (recoveryTrajectory.finished())
    {
        recovering = false;
        return;
    }
    else
    {
        recoveryTrajectory.update(dt);
    }

    const auto& recoveryFoot = recoverySupportPhase == Bipedal::SUPPORT_LEFT ? rightFoot : leftFoot;
    const auto& standingFoot = recoverySupportPhase == Bipedal::SUPPORT_LEFT ? leftFoot  : rightFoot;

    const auto& initialRecoveryFootPose = recoverySupportPhase == Bipedal::SUPPORT_LEFT ? initialRightFootPose : initialLeftFootPose;
    const auto& initialStandingFootPose = recoverySupportPhase == Bipedal::SUPPORT_LEFT ? initialLeftFootPose  : initialRightFootPose;

    if (recoverySupportPhase == Bipedal::SUPPORT_LEFT)
    {
        Eigen::Matrix4f recoveryFootPose = projectPoseToGround(initialRightFootPose);
        recoveryFootPose.block(0, 3, 3, 1) = recoveryTrajectory.position;
        if (recoveryFoot->getGlobalPose()(2, 3) < 5)
        {
            recoveryFootPose.block(0, 3, 3, 1) = rightFootPose.block(0, 3, 3, 1);
            if (recoveryFoot->getGlobalPose()(2, 3) > 0.5)
            {
                recoveryFootPose(2, 3) -= 0.1;
            }
        }
        leftFootPose = projectPoseToGround(initialStandingFootPose)
                     * leftFootPostureController->correctPosture(projectPoseToGround(initialStandingFootPose),
                                                                 standingFoot->getGlobalPose());
        rightFootPose = recoveryFootPose * rightFootPostureController->correctPosture(recoveryFootPose,
                                                                                      recoveryFoot->getGlobalPose());
    }
    else
    {
        Eigen::Matrix4f recoveryFootPose = projectPoseToGround(initialLeftFootPose);
        recoveryFootPose.block(0, 3, 3, 1) = recoveryTrajectory.position;
        // This is a hack: If we are above ground, move very slowly.
        // This should be replaced by propper cartesian space controll
        // of the foot position.
        if (recoveryFoot->getGlobalPose()(2, 3) < 5)
        {
            recoveryFootPose.block(0, 3, 3, 1) = leftFootPose.block(0, 3, 3, 1);
            if (recoveryFoot->getGlobalPose()(2, 3) > 0.5)
            {
                recoveryFootPose(2, 3) -= 0.1;
            }
        }
        rightFootPose = projectPoseToGround(initialStandingFootPose)
                     * rightFootPostureController->correctPosture(projectPoseToGround(initialStandingFootPose),
                                                                  standingFoot->getGlobalPose());
        leftFootPose = recoveryFootPose * leftFootPostureController->correctPosture(recoveryFootPose,
                                                                                    recoveryFoot->getGlobalPose());
    }
}

bool CapturePointRecovery::isRecovering() const
{
    return recovering;
}

void CapturePointRecovery::startRecovering(Bipedal::SupportPhase phase)
{
    recovering = true;
    initialLeftFootPose  = leftFoot->getGlobalPose();
    initialRightFootPose = leftFoot->getGlobalPose();
    initialPelvisPose    = pelvis->getGlobalPose();
    initialChestPose     = chest->getGlobalPose();
    leftFootPose = initialLeftFootPose;
    rightFootPose = initialRightFootPose;
    chestPose = initialChestPose;
    pelvisPose = initialPelvisPose;

    // we need to chose which foot we will use to recover
    if (phase == Bipedal::SUPPORT_BOTH)
    {
        const auto& iCP = computeImmediateCapturePoint(comPosition/1000.0, comVelocity/1000.0, gravity) * 1000.0;
        const double distRight = (iCP - rightFoot->getGlobalPose().block(0, 3, 3, 1)).norm();
        const double distLeft  = (iCP - leftFoot->getGlobalPose().block(0, 3, 3, 1)).norm();
        recoverySupportPhase = (distLeft > distRight) ? Bipedal::SUPPORT_LEFT : Bipedal::SUPPORT_RIGHT;
    }
    // in single support we can only use the non-supporting one
    else
    {
        recoverySupportPhase = phase;
    }
    const auto& recoveryFoot = recoverySupportPhase == Bipedal::SUPPORT_LEFT ? rightFoot : leftFoot;
    const auto& standingFoot = recoverySupportPhase == Bipedal::SUPPORT_LEFT ? leftFoot  : rightFoot;

    const Eigen::Vector3f& start = recoveryFoot->getGlobalPose().block(0, 3, 3, 1);
    const Eigen::Vector3f& end   = computeFutureCapturePoint(comPosition/1000.0,
                                                             comVelocity/1000.0,
                                                             standingFoot->getGlobalPose().block(0, 3, 3, 1)/1000.0,
                                                             gravity,
                                                             minTime) * 1000.0;
    Eigen::Vector3f diff = end - start;
    diff.z() = 0;
    //diff.normalize();
    Eigen::Vector3f h1(start.x(), start.y(), minHeight);
    Eigen::Vector3f h2(end.x(), end.y(), minHeight/4);
    //Eigen::Vector3f h2 = end - diff/2;

    recoveryTrajectory = Bipedal::CubivBezierCurve3f(start, end, h1, h2, minTime);
}

}

