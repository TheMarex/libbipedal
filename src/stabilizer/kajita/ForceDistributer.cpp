
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

ForceDistributor::ForceTorque ForceDistributor::distributeZMP(const Eigen::Vector3f& leftAnklePosition,
                                                              const Eigen::Vector3f& rightAnklePosition,
                                                              const Eigen::Matrix4f& leftFootTCP,
                                                              const Eigen::Matrix4f& rightFootTCP,
                                                              const Eigen::Vector3f& refZMP,
                                                              Kinematics::SupportPhase phase)
{
    Eigen::Matrix4f groundPoseLeft  = Kinematics::projectPoseToGround(leftFootTCP);
    Eigen::Matrix4f groundPoseRight = Kinematics::projectPoseToGround(rightFootTCP);
    Eigen::Vector3f groundRefZMP(refZMP.x(), refZMP.y(), 0);
    Eigen::Vector3f localZMPLeft    = VirtualRobot::MathTools::transformPosition(groundRefZMP, groundPoseLeft.inverse());
    Eigen::Vector3f localZMPRight   = VirtualRobot::MathTools::transformPosition(groundRefZMP, groundPoseRight.inverse());
    Eigen::Vector3f localAnkleLeft  = VirtualRobot::MathTools::transformPosition(
                                            Eigen::Vector3f(leftAnklePosition.x(), leftAnklePosition.y(), 0),
                                            groundPoseLeft.inverse());
    Eigen::Vector3f localAnkleRight = VirtualRobot::MathTools::transformPosition(
                                            Eigen::Vector3f(rightAnklePosition.x(), rightAnklePosition.y(), 0),
                                            groundPoseRight.inverse());


    double alpha = computeAlpha(groundPoseLeft, groundPoseRight, groundRefZMP, localZMPLeft.head(2), localZMPRight.head(2), phase);

    ForceTorque ft;
    // kg*m/s^2 = N
    ft.leftForce  = -(1-alpha) * mass * gravity;
    ft.rightForce = -alpha     * mass * gravity;

    // Note we need force as kg*mm/s^2
    ft.leftTorque  = (localAnkleLeft  - localZMPLeft).cross(ft.leftForce * 1000);
    ft.rightTorque = (localAnkleRight - localZMPRight).cross(ft.rightForce * 1000);

    // ZMP not contained in convex polygone
    if (std::fabs(alpha) > std::numeric_limits<float>::epsilon()
     && std::fabs(alpha-1) > std::numeric_limits<float>::epsilon())
    {
        Eigen::Vector3f leftTorqueWorld  = groundPoseLeft.block(0, 0, 3, 3)  * ft.leftTorque;
        Eigen::Vector3f rightTorqueWorld = groundPoseRight.block(0, 0, 3, 3) * ft.rightTorque;
        Eigen::Vector3f tau0 = -1 * (leftTorqueWorld + rightTorqueWorld);

        // Note: Our coordinate system is different than in the paper!
        Eigen::Vector3f xAxis = leftAnklePosition - rightAnklePosition;
        xAxis /= xAxis.norm();
        Eigen::Vector3f zAxis(0, 0, 1);
        Eigen::Vector3f yAxis = zAxis.cross(xAxis);
        yAxis /= yAxis.norm();
        Eigen::Matrix3f centerFrame;
        centerFrame.block(0, 0, 3, 1) = xAxis;
        centerFrame.block(0, 1, 3, 1) = yAxis;
        centerFrame.block(0, 2, 3, 1) = zAxis;

        Eigen::Vector3f centerTau0 = centerFrame.transpose() * tau0;
        Eigen::Vector3f torqueLeftCenter;
        torqueLeftCenter.x() = (1-alpha)*centerTau0.x();
        torqueLeftCenter.y() = centerTau0.y() < 0 ? centerTau0.y() : 0;
        Eigen::Vector3f torqueRightCenter;
        torqueRightCenter.x() = alpha*centerTau0.x();
        torqueRightCenter.y() = centerTau0.y() < 0 ? 0 : centerTau0.y();

        // tcp <--- world <--- centerFrame
        ft.leftTorque  = groundPoseLeft.block(0, 0, 3, 3).transpose()  * centerFrame * torqueLeftCenter;
        ft.rightTorque = groundPoseRight.block(0, 0, 3, 3).transpose() * centerFrame * torqueRightCenter;
    }

    // for some reason the sensors give a torque that is 10 times the expected value
    const double torqueFactor = 10;
    // convert to Nm
    ft.leftTorque  *= torqueFactor / 1000.0 / 1000.0;
    ft.rightTorque *= torqueFactor / 1000.0 / 1000.0;

    return ft;
}
