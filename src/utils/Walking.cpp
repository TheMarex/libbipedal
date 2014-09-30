#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/MathTools.h>
#include "VirtualRobot/CollisionDetection/CollisionChecker.h"

#include "utils/Walking.h"

namespace Bipedal
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
        for (unsigned i =  0; i < hull->vertices.size(); i++)
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

        // Note: The TCP coordinate system in Armar4 uses y as forward direction
        Eigen::Matrix2f pose;
        pose << -centerToLeft.x(), walkingDirection.x(), -centerToLeft.y(), walkingDirection.y();

        return pose;
    }

    Eigen::Vector2f computeHullContactPoint(const Eigen::Vector2f p, const VirtualRobot::MathTools::ConvexHull2DPtr& hull)
    {
        double min = std::numeric_limits<double>::max();
        VirtualRobot::MathTools::Segment2D min_segment;
        for(const auto& segment : hull->segments)
        {
            double dist = VirtualRobot::MathTools::distPointSegment(hull->vertices[segment.id1],
                    hull->vertices[segment.id2], p);
            if (dist < min)
            {
                min = dist;
                min_segment = segment;
            }
        }
        return VirtualRobot::MathTools::nearestPointOnSegment(hull->vertices[min_segment.id1], hull->vertices[min_segment.id2], p);
    }

    VirtualRobot::MathTools::ConvexHull2DPtr computeConvexHull(const VirtualRobot::RobotNodePtr& foot,
                                                               const VirtualRobot::RobotNodePtr& tcp)
    {
        auto colModel = foot->getCollisionModel()->clone();
        Eigen::Matrix4f relPose = tcp->getGlobalPose().inverse() * colModel->getGlobalPose();
        colModel->setGlobalPose(relPose);
        auto hull = Bipedal::ComputeFootContact(colModel);
        Bipedal::CenterConvexHull(hull);
        return hull;
    }

}
