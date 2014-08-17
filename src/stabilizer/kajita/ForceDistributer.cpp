
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

/**
 * Warning: This only works if we have a position without a slope!
 */
double ForceDistributor::computeAlpha(const Eigen::Matrix4f& groundPoseLeft,
                                       const Eigen::Matrix4f& groundPoseRight,
                                       const Eigen::Vector3f& refZMP,
                                       const Eigen::Vector2f& relZMPLeft,
                                       const Eigen::Vector2f& relZMPRight,
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
            // get contact points in world coordinates
            leftContact  = VirtualRobot::MathTools::transformPosition(Walking::computeHullContactPoint(relZMPLeft, leftConvexHull), groundPoseLeft);
            rightContact = VirtualRobot::MathTools::transformPosition(Walking::computeHullContactPoint(relZMPRight, rightConvexHull), groundPoseRight);
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
    Eigen::Matrix4f groundPoseLeft  = Kinematics::projectPoseToGround(leftAnklePose);
    Eigen::Matrix4f groundPoseRight = Kinematics::projectPoseToGround(rightAnklePose);
    Eigen::Vector2f relZMPLeft      = VirtualRobot::MathTools::transformPosition((Eigen::Vector2f) refZMP.head(2), groundPoseLeft.inverse());
    Eigen::Vector2f relZMPRight     = VirtualRobot::MathTools::transformPosition((Eigen::Vector2f) refZMP.head(2), groundPoseRight.inverse());

    // for some reason the sensors give a troque that is 10 times the expected value
    const double torqueFactor = 10;

    double alpha = computeAlpha(groundPoseLeft, groundPoseRight, refZMP, relZMPLeft, relZMPRight, phase);

    ForceTorque ft;
    ft.leftForce  = -(1-alpha) * mass * gravity;
    ft.rightForce = -alpha     * mass * gravity;

    Eigen::Vector3f relZMP;

    // Note: torque is in **local** coordinate system: origin at the ankle.
    relZMP << relZMPLeft.x()/1000.0f, relZMPLeft.y()/1000.0f, 0.0;
    ft.leftTorque  = relZMP.cross(ft.leftForce) * torqueFactor;
    relZMP << relZMPRight.x()/1000.0f, relZMPRight.y()/1000.0f, 0.0;
    ft.rightTorque = relZMP.cross(ft.rightForce) * torqueFactor;

    return ft;
}
