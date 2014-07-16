#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/MathTools.h>
#include "VirtualRobot/CollisionDetection/CollisionChecker.h"

#include "utils/Walking.h"

namespace Walking
{

    VirtualRobot::MathTools::ConvexHull2DPtr ComputeFootContact(const VirtualRobot::CollisionModelPtr& colModel)
    {
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

        return hull;
    }

    Eigen::Vector2f CenterConvexHull(const VirtualRobot::MathTools::ConvexHull2DPtr& hull)
    {
        Eigen::Vector2f center = VirtualRobot::MathTools::getConvexHullCenter(hull);

        // translate points of FootShape so, that center of convex hull is (0|0)
        for (int i =  0; i < hull->vertices.size(); i++)
        {
            hull->vertices[i] -= center;
        }

        return center;
    }

    Eigen::Matrix2f ComputeWalkingDirection(const Eigen::Vector2f& leftFootCenter, const Eigen::Vector2f& rightFootCenter)
    {
        Eigen::Vector2f center = (leftFootCenter + rightFootCenter) * 0.5;
        Eigen::Vector2f centerToLeft = (leftFootCenter - center);
        centerToLeft.normalize();
        Eigen::Matrix2f rotNinety;
        rotNinety << 0, 1, -1, 0;
        Eigen::Vector2f walkingDirection = rotNinety * centerToLeft;

        Eigen::Matrix2f pose;
        pose << walkingDirection.x(), centerToLeft.x(), walkingDirection.y(), centerToLeft.y();

        return pose;
    }

}
