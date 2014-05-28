#ifndef __WALKING_H__
#define __WALKING_H__

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/MathTools.h>
#include "VirtualRobot/CollisionDetection/CollisionChecker.h"

namespace Walking
{
/*
 * Returns the convex hull of the contact points centered at (0, 0).
 *
 * Center of the convex hull in the global space is returned in footCenter.
 *
 * All units are in mm.
 */
VirtualRobot::MathTools::ConvexHull2DPtr ComputeFootContact(VirtualRobot::RobotPtr robot, const std::string& footName, Eigen::Vector2f& footCenter)
{
    // get the Robot Nodes for the Feet
    VirtualRobot::RobotNodePtr foot = robot->getRobotNode(footName);
    if (!foot)
    {
        std::cout << "Cannot compute foot shape, because foot was not found!" << std::endl;
        return VirtualRobot::MathTools::ConvexHull2DPtr();
    }

    // get collision model of the feet
    VirtualRobot::CollisionModelPtr colModel = foot->getCollisionModel();
    if (!colModel)
    {
        std::cout << "Cannot compute foot shape, because the collision shape could not be retrieved!" << std::endl;
        return VirtualRobot::MathTools::ConvexHull2DPtr();
    }

    // let the feet collide with the floor and get the collision points
    VirtualRobot::MathTools::Plane plane =  VirtualRobot::MathTools::getFloorPlane();
    VirtualRobot::CollisionCheckerPtr colChecker = VirtualRobot::CollisionChecker::getGlobalCollisionChecker();
    std::vector< VirtualRobot::MathTools::ContactPoint > pointsFoot;
    std::vector< Eigen::Vector2f > points2D;

    // let the collision begin
    colChecker->getContacts(plane, colModel, pointsFoot, 5.0f);
    // project the points on the floor
    for (size_t u = 0; u < pointsFoot.size(); u++)
    {
        Eigen::Vector2f pt2d = VirtualRobot::MathTools::projectPointToPlane2D(pointsFoot[u].p, plane);
        points2D.push_back(pt2d);
    }

    // calculate the convex hulls and the appropriate centers
    VirtualRobot::MathTools::ConvexHull2DPtr hull = VirtualRobot::MathTools::createConvexHull2D(points2D);
    footCenter = VirtualRobot::MathTools::getConvexHullCenter(hull);

    // translate points of FootShape so, that center of convex hull is (0|0)
    for (int i =  0; i < hull->vertices.size(); i++)
        hull->vertices[i] -= footCenter;

    return hull;
}

// Get walking direction as 2D oriantation
// Units: mm
Eigen::Matrix2f ComputeWalkingDirection(const Eigen::Vector2f& leftFootCenter, const Eigen::Vector2f& rightFootCenter)
{
    Eigen::Vector2f center = (leftFootCenter + rightFootCenter) * 0.5;
    Eigen::Vector2f centerToLeft = (leftFootCenter - center);
    centerToLeft.normalize();
    Eigen::Matrix2f rotNinety;
    rotNinety << 0, 1, -1, 0;
    Eigen::Vector2f walkingDirection =  rotNinety * centerToLeft;

    Eigen::Matrix2f pose;
    pose << walkingDirection.x(), centerToLeft.x(), walkingDirection.y(), centerToLeft.y();
    return pose;
}
}

#endif
