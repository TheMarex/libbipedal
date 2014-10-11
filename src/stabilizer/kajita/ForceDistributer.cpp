
#include "stabilizer/kajita/ForceDistributor.h"

#include "utils/Walking.h"
#include "utils/Kinematics.h"

namespace Bipedal
{

ForceDistributor::ForceDistributor(double mass, Eigen::Vector3f gravity,
                 VirtualRobot::RobotNodePtr leftFoot,
                 VirtualRobot::RobotNodePtr rightFoot,
                 VirtualRobot::RobotNodePtr leftFootTCP,
                 VirtualRobot::RobotNodePtr rightFootTCP)
: mass(mass)
, gravity(gravity)
{
    leftConvexHull  = Bipedal::computeConvexHull(leftFoot, leftFootTCP);
    rightConvexHull = Bipedal::computeConvexHull(rightFoot, rightFootTCP);
}

/**
 * Warning: This only works if we have a position without a slope!
 */
double ForceDistributor::computeAlpha(const Eigen::Matrix4f& groundPoseLeft,
                                       const Eigen::Matrix4f& groundPoseRight,
                                       const Eigen::Vector3f& refZMP,
                                       const Eigen::Vector2f& relZMPLeft,
                                       const Eigen::Vector2f& relZMPRight,
                                       Bipedal::SupportPhase phase)
{
    double alpha;
    Eigen::Vector2f leftContact, rightContact;
    Eigen::Vector2f pAlpha;
    switch (phase)
    {
        case Bipedal::SUPPORT_LEFT:
            alpha = 0.0;
            break;
        case Bipedal::SUPPORT_RIGHT:
            alpha = 1.0;
            break;
        case Bipedal::SUPPORT_BOTH:
            // get contact points in world coordinates
            leftContact  = VirtualRobot::MathTools::transformPosition(Bipedal::computeHullContactPoint(relZMPLeft, leftConvexHull), groundPoseLeft);
            rightContact = VirtualRobot::MathTools::transformPosition(Bipedal::computeHullContactPoint(relZMPRight, rightConvexHull), groundPoseRight);
            pAlpha = VirtualRobot::MathTools::nearestPointOnSegment(
                            leftContact, rightContact,
                            Eigen::Vector2f(refZMP.head(2))
                     );
            alpha = (leftContact - pAlpha).norm() / (leftContact - rightContact).norm();
            break;
    }
    return alpha;
}

ForceDistributor::ForceTorque ForceDistributor::distributeZMP(const Eigen::Vector3f& localAnkleLeft,
                                                              const Eigen::Vector3f& localAnkleRight,
                                                              const Eigen::Matrix4f& leftFootPoseGroundFrame,
                                                              const Eigen::Matrix4f& rightFootPoseGroundFrame,
                                                              const Eigen::Vector3f& zmpRefGroundFrame,
                                                              Bipedal::SupportPhase phase)
{
    Eigen::Matrix4f groundPoseLeft  = Bipedal::projectPoseToGround(leftFootPoseGroundFrame);
    Eigen::Matrix4f groundPoseRight = Bipedal::projectPoseToGround(rightFootPoseGroundFrame);
    Eigen::Vector3f localZMPLeft    = VirtualRobot::MathTools::transformPosition(zmpRefGroundFrame, groundPoseLeft.inverse());
    Eigen::Vector3f localZMPRight   = VirtualRobot::MathTools::transformPosition(zmpRefGroundFrame, groundPoseRight.inverse());

    double alpha = computeAlpha(groundPoseLeft, groundPoseRight, zmpRefGroundFrame, localZMPLeft.head(2), localZMPRight.head(2), phase);

    //std::cout << "########## " << alpha << " ###########" << std::endl;

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
        Eigen::Vector3f leftTorqueGroundFrame  = groundPoseLeft.block(0, 0, 3, 3)  * ft.leftTorque;
        Eigen::Vector3f rightTorqueGroundFrame = groundPoseRight.block(0, 0, 3, 3) * ft.rightTorque;
        Eigen::Vector3f tau0 = -1 * (leftTorqueGroundFrame + rightTorqueGroundFrame);

        //std::cout << "Tau0World: " << tau0.transpose() << std::endl;
        //std::cout << "leftTorqueWorld: "  << leftTorqueWorld.transpose() << std::endl;
        //std::cout << "rightTorqueWorld: " << rightTorqueWorld.transpose() << std::endl;

        // Note: Our coordinate system is different than in the paper!
        // Also this is not the same as the ground frame.
        Eigen::Vector3f xAxis = leftFootPoseGroundFrame.block(0,3,3,1) + localAnkleLeft
                              - localAnkleRight - rightFootPoseGroundFrame.block(0,3,3,1);
        xAxis /= xAxis.norm();
        Eigen::Vector3f zAxis(0, 0, 1);
        Eigen::Vector3f yAxis = zAxis.cross(xAxis);
        yAxis /= yAxis.norm();
        Eigen::Matrix3f centerFrame;
        centerFrame.block(0, 0, 3, 1) = xAxis;
        centerFrame.block(0, 1, 3, 1) = yAxis;
        centerFrame.block(0, 2, 3, 1) = zAxis;

        //std::cout << "Center frame:\n" << centerFrame << std::endl;

        Eigen::Vector3f centerTau0 = centerFrame.transpose() * tau0;
        Eigen::Vector3f leftTorqueCenter;
        leftTorqueCenter.x() = (1-alpha)*centerTau0.x();
        leftTorqueCenter.y() = centerTau0.y() < 0 ? centerTau0.y() : 0;
        leftTorqueCenter.z() = 0;
        Eigen::Vector3f rightTorqueCenter;
        rightTorqueCenter.x() = alpha*centerTau0.x();
        rightTorqueCenter.y() = centerTau0.y() < 0 ? 0 : centerTau0.y();
        rightTorqueCenter.z() = 0;

        //std::cout << "Tau0Center: " << centerTau0.transpose() << std::endl;
        //std::cout << "leftTorqueCenter: "  << leftTorqueCenter.transpose() << std::endl;
        //std::cout << "rightTorqueCenter: " << rightTorqueCenter.transpose() << std::endl;

        // tcp <--- ground frame <--- center frame
        ft.leftTorque  = groundPoseLeft.block(0, 0, 3, 3).transpose()  * centerFrame * leftTorqueCenter;
        ft.rightTorque = groundPoseRight.block(0, 0, 3, 3).transpose() * centerFrame * rightTorqueCenter;
    }

    // Torque depends on timestep, we need a way to do this correctly.
    const double torqueFactor = 1;
    // convert to Nm
    ft.leftTorque  *= torqueFactor / 1000.0 / 1000.0;
    ft.rightTorque *= torqueFactor / 1000.0 / 1000.0;

    //std::cout << "leftTorque: "  << ft.leftTorque.transpose() << std::endl;
    //std::cout << "rightTorque: " << ft.rightTorque.transpose() << std::endl;

    return ft;
}

}
