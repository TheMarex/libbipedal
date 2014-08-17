#ifndef __WALKING_H__
#define __WALKING_H__

#include <VirtualRobot/MathTools.h>

namespace Walking
{
    /*
     * Returns the convex hull of the contact points in global coordinates.
     *
     * All units are in mm.
     */
    VirtualRobot::MathTools::ConvexHull2DPtr ComputeFootContact(const VirtualRobot::CollisionModelPtr& colModel);

    /**
     * Shift convex hull to origin
     * and return old center in global coordinates
     */
    Eigen::Vector2f CenterConvexHull(const VirtualRobot::MathTools::ConvexHull2DPtr& hull);

    /**
     * Get walking direction as 2D oriantation
     * Units: mm
     */
    Eigen::Matrix2f ComputeWalkingDirection(const Eigen::Vector2f& leftFootCenter, const Eigen::Vector2f& rightFootCenter);

    /**
     * Computes contact point on the convex hull.
     * Expects convex hull segments and p to be in the same coordinate system.
     * Note: If the contact point is inside the convex hull, you will still get a correct contact point.
     *
     * Returns contact point on convex hull segment in the above coordinate system.
     */
    Eigen::Vector2f computeHullContactPoint(const Eigen::Vector2f p, const VirtualRobot::MathTools::ConvexHull2DPtr& hull);
}

#endif
