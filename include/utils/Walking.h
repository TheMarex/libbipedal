/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __WALKING_H__
#define __WALKING_H__

#include "../bipedal.h"

#include <VirtualRobot/MathTools.h>

namespace Bipedal
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
     * Trans convex hull by the given homogenous 3x3 2D transformation matrix.
     */
    void TransformConvexHull(const VirtualRobot::MathTools::ConvexHull2DPtr& hull, const Eigen::Matrix3f& transformation);

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
     *
     * FIXME: Should be moved to MathTools?
     */
    Eigen::Vector2f computeHullContactPoint(const Eigen::Vector2f p, const VirtualRobot::MathTools::ConvexHull2DPtr& hull);

    /**
     * Computes the convex hull of the support polygone the given body.
     * Centers it around the TCP.
     */
    VirtualRobot::MathTools::ConvexHull2DPtr computeConvexHull(const VirtualRobot::RobotNodePtr& foot,
                                                               const VirtualRobot::RobotNodePtr& tcp);

    /**
     * Computes the support polygone given the convex hull of the contact point of both
     * feet and the support phase.
     *
     * Convex hull is centered around the ground frame of the support phase.
     */
    VirtualRobot::MathTools::ConvexHull2DPtr computeSupportPolygone(const Eigen::Matrix4f& leftFootPose,
                                                                    const Eigen::Matrix4f& rightFootPose,
                                                                    const VirtualRobot::MathTools::ConvexHull2DPtr& leftFootHull,
                                                                    const VirtualRobot::MathTools::ConvexHull2DPtr& rightFootHull,
                                                                    Bipedal::SupportPhase phase);
}

#endif
