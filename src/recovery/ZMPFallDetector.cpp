#include "bipedal.h"
#include "recovery/ZMPFallDetector.h"

#include "utils/Kinematics.h"
#include "utils/Estimation.h"
#include "utils/Walking.h"
#include "utils/Interpolation.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/MathTools.h>

#include <boost/make_shared.hpp>

namespace Bipedal
{

ZMPFallDetector::ZMPFallDetector(const VirtualRobot::RobotNodePtr& leftFoot,
                                 const VirtualRobot::RobotNodePtr& rightFoot,
                                 const VirtualRobot::ContactSensorPtr& leftFootContactSensor,
                                 const VirtualRobot::ContactSensorPtr& rightFootContactSensor,
                                 const VirtualRobot::MathTools::ConvexHull2DPtr& leftSupportHull,
                                 const VirtualRobot::MathTools::ConvexHull2DPtr& rightSupportHull)
: zmpEstimator(9.81)
, leftFoot(leftFoot)
, rightFoot(rightFoot)
, leftSupportHull(leftSupportHull)
, rightSupportHull(rightSupportHull)
, supportPhaseSensor(new SupportPhaseSensor(leftFootContactSensor, rightFootContactSensor))
, maxHullDist(10)
, minFallingFrames(10)
, fallingFrameCounter(0)
, lastSupportPhase(Bipedal::SUPPORT_NONE)
, contactPoint(Eigen::Vector3f::Zero())
, falling(false)
{
}

const Eigen::Vector3f& ZMPFallDetector::getContactPoint() const { return contactPoint; }

bool ZMPFallDetector::isFalling() const
{
    return falling;
}

bool ZMPFallDetector::getStabilityInidcator(SupportPhase phase,
                                            const Eigen::Matrix4f& leftFootPose,
                                            const Eigen::Matrix4f& rightFootPose)
{
    // If not foot touches the ground, we are fucked.
    bool stillFalling = phase == SUPPORT_NONE;

    if (stillFalling)
        return true;

    const auto& supportHull = [this, phase, &leftFootPose, &rightFootPose]() {
        if (phase == Bipedal::SUPPORT_LEFT)
            return leftSupportHull;
        if (phase == Bipedal::SUPPORT_RIGHT)
            return rightSupportHull;

        BOOST_ASSERT(phase == Bipedal::SUPPORT_BOTH);

        // Foot positions changed
        if (lastSupportPhase != phase)
        {
            dualSupportHull = computeSupportPolygone(leftFootPose,
                                                     rightFootPose,
                                                     leftSupportHull,
                                                     rightSupportHull,
                                                     phase);
        }
        lastSupportPhase = phase;

        return dualSupportHull;
    }();
    // center of convex hull and orientation should correspond with ground frame
    const auto& groundFrame = Bipedal::computeGroundFrame(leftFootPose, rightFootPose, phase);
    Eigen::Vector3f zmp = Eigen::Vector3f::Zero();
    zmp.head(2) = zmpEstimator.estimation * 1000;

    Eigen::Vector3f zmpConvexHull = VirtualRobot::MathTools::transformPosition(zmp, groundFrame.inverse());

    // if zmp is inside of the CH, we are not falling
    if (VirtualRobot::MathTools::isInside(zmpConvexHull.head(2), supportHull))
    {
        stillFalling = false;
        std::cout << "(" << phase << ") ZMP is inside the CH. " << std::endl;
    }
    else
    {
        Eigen::Vector3f contact = Eigen::Vector3f::Zero();
        contact.head(2) = Bipedal::computeHullContactPoint(zmpConvexHull.head(2), supportHull);

        contactPoint = VirtualRobot::MathTools::transformPosition(contact, groundFrame);

        double dist = (contact-zmpConvexHull).norm();
        std::cout << "(" << phase << ") ZMP is outside the CH: " << dist << std::endl;
        stillFalling = dist > maxHullDist;
    }

    return stillFalling;
}

void ZMPFallDetector::update(const Eigen::Vector3f& com, const Eigen::Vector3f& comVel, double dt)
{
    supportPhaseSensor->update(dt);
    SupportPhase phase = supportPhaseSensor->phase;

    // update the dynamics estimation
    zmpEstimator.update(com/1000.0,
                        comVel/1000.0, dt);

    bool stillFalling = getStabilityInidcator(phase,
                                              leftFoot->getGlobalPose(),
                                              rightFoot->getGlobalPose());


    if (stillFalling && !falling)
    {
        fallingFrameCounter++;
        if (fallingFrameCounter > minFallingFrames)
        {
            falling = true;
        }
    }

    if (!stillFalling)
    {
        if (fallingFrameCounter > minFallingFrames/2.0)
        {
            fallingFrameCounter--;
        }
        else
        {
            fallingFrameCounter = 0;
            falling = false;
        }
    }
}

}

