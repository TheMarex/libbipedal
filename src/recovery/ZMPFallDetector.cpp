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

ZMPFallDetector::ZMPFallDetector(const VirtualRobot::RobotNodeSetPtr& colModelNodes,
                                 const VirtualRobot::RobotNodePtr& leftFoot,
                                 const VirtualRobot::RobotNodePtr& rightFoot,
                                 const VirtualRobot::ContactSensorPtr& leftFootContactSensor,
                                 const VirtualRobot::ContactSensorPtr& rightFootContactSensor,
                                 const VirtualRobot::MathTools::ConvexHull2DPtr& leftSupportHull,
                                 const VirtualRobot::MathTools::ConvexHull2DPtr& rightSupportHull)
: colModelNodes(colModelNodes)
, zmpEstimator(9.81)
, leftFoot(leftFoot)
, rightFoot(rightFoot)
, leftSupportHull(leftSupportHull)
, rightSupportHull(rightSupportHull)
, supportPhaseSensor(new SupportPhaseSensor(leftFootContactSensor, rightFootContactSensor))
, maxHullDist(100)
, minFallingFrames(10)
, fallingFrameCounter(0)
, lastSupportPhase(Bipedal::SUPPORT_NONE)
, contactPoint(Eigen::Vector3f::Zero())
, falling(false)
{
}

/*
VirtualRobot::MathTools::ConvexHull2DPtr ZMPFallDetector::getLeftSupportPolygone() const
{
    auto transformedHull = boost::make_shared<VirtualRobot::MathTools::ConvexHull2D>(*leftSupportHull);
    Eigen::Matrix3f transformationMatrix;
    const auto& groundFrame = Bipedal::computeGroundFrame(leftFoot->getGlobalPose(), rightFoot->getGlobalPose(), SUPPORT_LEFT);
    transformationMatrix.block(0, 0, 2, 2) = groundFrame.block(0, 0, 2, 2);
    transformationMatrix.block(0, 2, 2, 1) = groundFrame.block(0, 3, 2, 1);
    Bipedal::TransformConvexHull(transformedHull, transformationMatrix);
    return transformedHull;
}
*/

const Eigen::Vector3f& ZMPFallDetector::getContactPoint() const { return contactPoint; }

bool ZMPFallDetector::isFalling() const
{
    return falling;
}

bool ZMPFallDetector::getStabilityInidcator(SupportPhase phase)
{
    // If not foot touches the ground, we are fucked.
    bool stillFalling = phase == SUPPORT_NONE;

    if (stillFalling)
        return true;

    const auto& supportHull = [this, phase]() {
        if (phase == Bipedal::SUPPORT_LEFT)
            return leftSupportHull;
        if (phase == Bipedal::SUPPORT_RIGHT)
            return rightSupportHull;

        BOOST_ASSERT(phase == Bipedal::SUPPORT_BOTH);

        // Foot positions changed
        if (lastSupportPhase != phase)
        {
            dualSupportHull = computeSupportPolygone(leftFoot->getGlobalPose(),
                                                     rightFoot->getGlobalPose(),
                                                     leftSupportHull,
                                                     rightSupportHull,
                                                     phase);
        }
        lastSupportPhase = phase;

        return dualSupportHull;
    }();
    // center of convex hull and orientation should correspond with ground frame
    const auto& groundFrame = Bipedal::computeGroundFrame(leftFoot->getGlobalPose(), rightFoot->getGlobalPose(), phase);
    Eigen::Vector3f zmp = Eigen::Vector3f::Zero();
    zmp.head(2) = zmpEstimator.estimation;

    Eigen::Vector3f zmpConvexHull = VirtualRobot::MathTools::transformPosition(zmp, groundFrame.inverse());

    // if zmp is inside of the CH, we are not falling
    if (VirtualRobot::MathTools::isInside(zmpConvexHull.head(2), supportHull))
    {
        stillFalling = false;
    }
    else
    {
        Eigen::Vector3f contact = Eigen::Vector3f::Zero();
        contact.head(2) = Bipedal::computeHullContactPoint(zmpConvexHull.head(2), supportHull);

        contactPoint = VirtualRobot::MathTools::transformPosition(contact, groundFrame);

        double dist = (contact-zmpConvexHull).norm();
        stillFalling = dist > maxHullDist;
    }

    return stillFalling;
}

void ZMPFallDetector::update(double dt)
{
    supportPhaseSensor->update(dt);
    SupportPhase phase = supportPhaseSensor->phase;

    // update the dynamics estimation
    const auto& com = colModelNodes->getCoM();
    zmpEstimator.update(com, dt);

    bool stillFalling = getStabilityInidcator(phase);

    if (stillFalling && !falling)
    {
        fallingFrameCounter++;
        if (fallingFrameCounter > minFallingFrames)
        {
            falling = true;
        }
    }

    if (!stillFalling && falling)
    {
        if (fallingFrameCounter > 0)
        {
            fallingFrameCounter--;
        }
        else
        {
            falling = false;
        }
    }
}

}

