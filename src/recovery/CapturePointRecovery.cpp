#include "bipedal.h"
#include "recovery/CapturePointRecovery.h"

#include "utils/Kinematics.h"
#include "utils/Estimation.h"
#include "utils/Walking.h"
#include "utils/Interpolation.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/MathTools.h>

CapturePointRecovery::CapturePointRecovery(const VirtualRobot::RobotNodeSetPtr& colModelNodes,
                                           const VirtualRobot::RobotNodePtr& leftFoot,
                                           const VirtualRobot::RobotNodePtr& rightFoot,
                                           const VirtualRobot::RobotNodePtr& chest,
                                           const VirtualRobot::RobotNodePtr& pelvis,
                                           const VirtualRobot::MathTools::ConvexHull2DPtr& leftSupportHull,
                                           const VirtualRobot::MathTools::ConvexHull2DPtr& rightSupportHull)
: colModelNodes(colModelNodes)
, velocityEstimator(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero())
, leftFoot(leftFoot)
, rightFoot(rightFoot)
, chest(chest)
, pelvis(pelvis)
, leftSupportHull(leftSupportHull)
, rightSupportHull(rightSupportHull)
, maxHullDist(50)
, minHeight(100)
, minTime(0.3)
, minFallingFrames(10)
, fallingFrameCounter(0)
, lastSupportPhase(Kinematics::SUPPORT_NONE)
, contactPoint(Eigen::Vector3f::Zero())
, state(STATE_INITIAL)
, leftFootPose(Eigen::Matrix4f::Identity())
, rightFootPose(Eigen::Matrix4f::Identity())
, chestPose(Eigen::Matrix4f::Identity())
, pelvisPose(Eigen::Matrix4f::Identity())
{
}

const Eigen::Matrix4f& CapturePointRecovery::getLeftFootPose() const { return leftFootPose; }
const Eigen::Matrix4f& CapturePointRecovery::getRightFootPose() const { return rightFootPose; }
const Eigen::Matrix4f& CapturePointRecovery::getChestPose() const { return chestPose; }
const Eigen::Matrix4f& CapturePointRecovery::getPelvisPose() const { return pelvisPose; }
Kinematics::SupportPhase CapturePointRecovery::getSupportPhase() const { return lastSupportPhase; }

const Eigen::Vector3f& CapturePointRecovery::getContactPoint() const { return contactPoint; }
const Bipedal::CubicBezierCurve3f& CapturePointRecovery::getCaptureTrajectory() const { return recoveryTrajectory; }

/**
 * Estimates the position of the iCP at time + t
 */
Eigen::Vector3f CapturePointRecovery::getFutureCapturePoint(const VirtualRobot::RobotNodePtr& ankleNode, double t) const
{
    const auto& com = colModelNodes->getCoM(); // in mm
    double g = 9.81 * 1000; // mm/s^2
    double omega0 = sqrt(g / com.z());

    // both in mm
    Eigen::Vector3f iCP   = getImmediateCapturePoint();
    Eigen::Vector3f ankle = Eigen::Vector3f::Zero();
    ankle.head(2) = ankleNode->getGlobalPose().block(0, 3, 2, 1);

    Eigen::Vector3f futureCP = (iCP -  ankle) * exp(t * omega0) + ankle;

    return futureCP;
}

/**
 * Get the current iCP
 */
Eigen::Vector3f CapturePointRecovery::getImmediateCapturePoint() const
{
    const auto& com = colModelNodes->getCoM(); // in mm
    const auto& comVel = velocityEstimator.estimation; // in mm/s
    double gravity = 9.81 * 1000; // in mm/s^2
    Eigen::Vector3f iCP = Eigen::Vector3f::Zero();
    iCP.head(2) = com.head(2) + comVel.head(2) * sqrt(com.z() / gravity);

    return iCP;
}

bool CapturePointRecovery::isFalling() const
{
    return state == STATE_FALLING;
}

void CapturePointRecovery::update(Kinematics::SupportPhase phase, double dt)
{
    // update the dynamics estimation
    const auto& com = colModelNodes->getCoM();
    velocityEstimator.update(com, dt);

    const auto& supportHull = [this, phase]() {
        if (phase == Kinematics::SUPPORT_LEFT)
            return leftSupportHull;
        if (phase == Kinematics::SUPPORT_RIGHT)
            return rightSupportHull;

        BOOST_ASSERT(phase == Kinematics::SUPPORT_BOTH);

        // Foot positions changed
        if (lastSupportPhase != phase)
        {
            recomputeDualSupportHull();
        }

        return dualSupportHull;
    }();
    lastSupportPhase = phase;
    // center of convex hull and orientation should correspond with ground frame
    const auto& groundFrame = Kinematics::computeGroundFrame(leftFoot->getGlobalPose(), rightFoot->getGlobalPose(), phase);
    const auto& iCP = getImmediateCapturePoint();

    Eigen::Vector3f iCPConvexHull = VirtualRobot::MathTools::transformPosition(iCP, groundFrame.inverse());

    bool falling;
    // if CP is inside of the CH, we are not falling
    if (VirtualRobot::MathTools::isInside(iCPConvexHull.head(2), supportHull))
    {
        falling = false;
    }
    else
    {
        Eigen::Vector3f contact = Eigen::Vector3f::Zero();
        contact.head(2) = Walking::computeHullContactPoint(iCPConvexHull.head(2), supportHull);

        contactPoint = VirtualRobot::MathTools::transformPosition(contact, groundFrame);

        double dist = (contact-iCPConvexHull).norm();
        falling = dist > maxHullDist;
    }

    if (falling && state == STATE_INITIAL)
    {
        fallingFrameCounter++;
        if (fallingFrameCounter > minFallingFrames)
        {
            state = STATE_FALLING;
            initializeRecovery(phase);
        }
    }

    if (!falling && state == STATE_FALLING && recoveryTrajectory.finished())
    {
        if (fallingFrameCounter > 0)
        {
            fallingFrameCounter--;
        }
        else
        {
            std::cout << "Recovered!" << std::endl;
            state = STATE_RECOVERED;
        }
    }

    if (state == STATE_FALLING)
    {
        if (recoveryTrajectory.finished())
        {
            state = STATE_FAILED;
            std::cout << "Recovery failed!" << std::endl;
        }
        else
        {
            const auto& recoveryFoot = recoverySupportPhase == Kinematics::SUPPORT_LEFT ? rightFoot : leftFoot;
            const auto& standingFoot = recoverySupportPhase == Kinematics::SUPPORT_LEFT ? leftFoot  : rightFoot;

            recoveryTrajectory.update(dt);

            Eigen::Matrix4f recoveryFootPose = recoveryFoot->getGlobalPose().inverse();
            recoveryFootPose.block(0, 3, 3, 1) = recoveryTrajectory.position;

            if (recoverySupportPhase == Kinematics::SUPPORT_LEFT)
            {
                rightFootPose = recoveryFootPose;
            }
            else
            {
                leftFootPose = recoveryFootPose;
            }
        }
    }
}

void CapturePointRecovery::initializeRecovery(Kinematics::SupportPhase phase)
{
    leftFootPose  = leftFoot->getGlobalPose();
    rightFootPose = leftFoot->getGlobalPose();
    pelvisPose    = pelvis->getGlobalPose();
    chestPose     = chest->getGlobalPose();

    // we need to chose which foot we will use to recover
    if (phase == Kinematics::SUPPORT_BOTH)
    {
        const auto& iCP = getImmediateCapturePoint();
        const double distRight = (iCP - rightFoot->getGlobalPose().block(0, 3, 3, 1)).norm();
        const double distLeft  = (iCP - leftFoot->getGlobalPose().block(0, 3, 3, 1)).norm();
        recoverySupportPhase = (distLeft > distRight) ? Kinematics::SUPPORT_LEFT : Kinematics::SUPPORT_RIGHT;
    }
    // in single support we can only use the non-supporting one
    else
    {
        recoverySupportPhase = phase;
    }
    const auto& recoveryFoot = recoverySupportPhase == Kinematics::SUPPORT_LEFT ? rightFoot : leftFoot;
    const auto& standingFoot = recoverySupportPhase == Kinematics::SUPPORT_LEFT ? leftFoot  : rightFoot;

    const Eigen::Vector3f& start = recoveryFoot->getGlobalPose().block(0, 3, 3, 1);
    const Eigen::Vector3f& end   = getFutureCapturePoint(standingFoot, minTime);
    //Eigen::Vector3f diff = end - start;
    //diff.z() = 0;
    //diff.normalize();
    Eigen::Vector3f h1(start.x(), start.y(), minHeight);
    Eigen::Vector3f h2(end.x(), end.y(), minHeight);
    //Eigen::Vector3f h2 = end - diff;

    recoveryTrajectory = Bipedal::CubivBezierCurve3f(start, end, h1, h2, minTime);
}

void CapturePointRecovery::recomputeDualSupportHull()
{
    Eigen::Vector2f offset = rightFoot->getGlobalPose().block(0, 3, 2, 1) - leftFoot->getGlobalPose().block(0, 3, 2, 1);

    std::vector<Eigen::Vector2f> points;
    for (const auto& v : leftSupportHull->vertices)
    {
        points.push_back(v - offset/2);
    }
    for (const auto& v : rightSupportHull->vertices)
    {
        points.push_back(v + offset/2);
    }

    dualSupportHull = VirtualRobot::MathTools::createConvexHull2D(points);
    Walking::CenterConvexHull(dualSupportHull);
}

