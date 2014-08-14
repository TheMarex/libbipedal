
#include "stabilizer/kajita/ForceDistributor.h"

#include "utils/Walking.h"
#include "utils/Kinematics.h"

ForceDistributor::ForceDistributor(double mass, Eigen::Vector3f gravity,
                 VirtualRobot::RobotNodePtr leftFoot,
                 VirtualRobot::RobotNodePtr rightFoot,
                 VirtualRobot::RobotNodePtr leftFootTCP,
                 VirtualRobot::RobotNodePtr rightFootTCP)
: mass(mass)
, gravity(gravity)
{
    leftConvexHull  = computeConvexHull(leftFoot, leftFootTCP);
    rightConvexHull = computeConvexHull(rightFoot, rightFootTCP);
}

VirtualRobot::MathTools::ConvexHull2DPtr ForceDistributor::computeConvexHull(const VirtualRobot::RobotNodePtr& foot,
                                                                             const VirtualRobot::RobotNodePtr& tcp)
{
    auto colModel = foot->getCollisionModel()->clone();
    Eigen::Matrix4f relPose = tcp->getGlobalPose().inverse() * colModel->getGlobalPose();
    colModel->setGlobalPose(relPose);
    auto hull = Walking::ComputeFootContact(colModel);
    Walking::CenterConvexHull(hull);
    return hull;
}

Eigen::Vector2f transform2D(const Eigen::Vector2f& pt, const Eigen::Matrix4f& pose)
{
    Eigen::Vector4f homWorldPt = pose * Eigen::Vector4f(pt.x(), pt.y(), 0, 1);
    return homWorldPt.block(0, 0, 2, 1);
}

Eigen::Vector2f ForceDistributor::computeHullContact(const Eigen::Matrix4f& anklePose,
                                                     const Eigen::Vector3f& refZMP,
                                                     const VirtualRobot::MathTools::ConvexHull2DPtr& hull)
{
    // FIXME don't use inverse - we need refZMP in local frame
    Eigen::Vector4f homRelZMP = anklePose.inverse() * Eigen::Vector4f(refZMP.x(), refZMP.y(), refZMP.z(), 1);
    Eigen::Vector2f relZMP = homRelZMP.head(2);
    double min = std::numeric_limits<double>::max();
    VirtualRobot::MathTools::Segment2D min_segment;
    for(const auto& segment : hull->segments)
    {
        double dist = VirtualRobot::MathTools::distPointSegment(hull->vertices[segment.id1],
                                          hull->vertices[segment.id2], relZMP);
        if (dist < min)
        {
            min = dist;
            min_segment = segment;
        }
    }
    return VirtualRobot::MathTools::nearestPointOnSegment(hull->vertices[min_segment.id1], hull->vertices[min_segment.id2], relZMP);
}

double ForceDistributor::computeAlpha(const Eigen::Matrix4f& leftAnklePose,
                                       const Eigen::Matrix4f& rightAnklePose,
                                       const Eigen::Vector3f& refZMP,
                                       Kinematics::SupportPhase phase)
{
    double alpha;
    Eigen::Vector2f leftContact, rightContact;
    Eigen::Vector2f pAlpha;
    switch (phase)
    {
        case Kinematics::SUPPORT_LEFT:
            alpha = 0.0;
            break;
        case Kinematics::SUPPORT_RIGHT:
            alpha = 1.0;
            break;
        case Kinematics::SUPPORT_BOTH:
            alpha = 0.5;
            // get contact points in world coordinates
            Eigen::Vector4f hom;
            hom(2, 0) = 0;
            hom(3, 0) = 1;
            hom.head(2) = computeHullContact(leftAnklePose, refZMP, leftConvexHull);
            leftContact  = (leftAnklePose  * hom).head(2);
            hom.head(2) = computeHullContact(rightAnklePose, refZMP, rightConvexHull);
            rightContact = (rightAnklePose * hom).head(2);
            pAlpha = VirtualRobot::MathTools::nearestPointOnSegment(
                            leftContact, rightContact,
                            Eigen::Vector2f(refZMP.head(2))
                     );
            alpha = (leftContact - pAlpha).norm() / (leftContact - rightContact).norm();
            break;
    }
    return alpha;
}

ForceDistributor::ForceTorque ForceDistributor::distributeZMP(const Eigen::Matrix4f& leftAnklePose,
                                                              const Eigen::Matrix4f& rightAnklePose,
                                                              const Eigen::Vector3f& refZMP,
                                                              Kinematics::SupportPhase phase)
{
    double alpha = computeAlpha(leftAnklePose, rightAnklePose, refZMP, phase);
    ForceTorque ft;
    ft.rightForce = -alpha     * mass * gravity;
    ft.leftForce  = -(1-alpha) * mass * gravity;
    Eigen::Vector3f zmp;
    zmp << refZMP.x()/1000.0f, refZMP.y()/1000.0f, 0.0;
    Eigen::Vector3f ankle = leftAnklePose.block(0, 3, 3, 1) / 1000.0f;
    ft.leftTorque  = (ankle - zmp).cross(ft.leftForce);
    ankle = rightAnklePose.block(0, 3, 3, 1) / 1000.0f;
    ft.rightTorque = (ankle - zmp).cross(ft.rightForce);

    return ft;
}
