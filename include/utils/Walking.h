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
}

#endif
