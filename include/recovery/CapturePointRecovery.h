#ifndef __CAPTURE_POINT_RECOVERY_H__
#define __CAPTURE_POINT_RECOVERY_H__

#include "PushRecovery.h"

#include "../utils/Kinematics.h"
#include "../utils/Estimation.h"
#include "../utils/Walking.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/MathTools.h>

class CapturePointRecovery : public PushRecovery
{
public:
    CapturePointRecovery(const VirtualRobot::RobotNodeSetPtr& nodes,
                         const VirtualRobot::RobotNodeSetPtr& colModelNodes,
                         const VirtualRobot::RobotNodeSetPtr& leftLegNodes,
                         const VirtualRobot::RobotNodeSetPtr& rightLegNodes,
                         const VirtualRobot::RobotNodePtr& leftFoot,
                         const VirtualRobot::RobotNodePtr& rightFoot,
                         const VirtualRobot::MathTools::ConvexHull2DPtr& leftSupportHull,
                         const VirtualRobot::MathTools::ConvexHull2DPtr& rightSupportHull)
    : nodes(nodes)
    , colModelNodes(colModelNodes)
    , velocityEstimator(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero())
    , immediateCapturePoint(Eigen::Vector3f::Zero())
    , leftLegIK(new VirtualRobot::DifferentialIK(leftLegNodes))
    , rightLegIK(new VirtualRobot::DifferentialIK(rightLegNodes))
    , leftFoot(leftFoot)
    , rightFoot(rightFoot)
    , leftSupportHull(leftSupportHull)
    , rightSupportHull(rightSupportHull)
    , maxHullDist(200)
    {
    }

    virtual bool isFalling(Kinematics::SupportPhase phase, double dt) override
    {
        updateCaputePoint(dt);

        // FIXME actually we could make this work with the convex hull of both support
        // polygones.
        if (phase == Kinematics::SUPPORT_BOTH)
        {
            return false;
        }

        const auto& supportFoot = phase == Kinematics::SUPPORT_LEFT ? leftFoot        : rightFoot;
        const auto& supportHull = phase == Kinematics::SUPPORT_LEFT ? leftSupportHull : rightSupportHull;

        Eigen::Vector3f iCPSupportFoot = VirtualRobot::MathTools::transformPosition(immediateCapturePoint, supportFoot->getGlobalPose().inverse());
        Eigen::Vector3f contact = Eigen::Vector3f::Zero();
        contact.head(2) = Walking::computeHullContactPoint(iCPSupportFoot.head(2), supportHull);

        contactPoint = VirtualRobot::MathTools::transformPosition(contact, supportFoot->getGlobalPose());

        double dist = (contact-iCPSupportFoot).norm();

        return dist > maxHullDist;
    }

    virtual bool computeRecoveryTrajectory(Kinematics::SupportPhase phase, Eigen::VectorXf& trajectory) override
    {
        if (phase == Kinematics::SUPPORT_BOTH)
        {
            return false;
        }

        const auto& tcp = phase == Kinematics::SUPPORT_LEFT ? rightFoot  : leftFoot;
        const auto& ik  = phase == Kinematics::SUPPORT_LEFT ? rightLegIK : leftLegIK;

        Eigen::VectorXf backup;
        nodes->getJointValues(backup);
        auto currentPose = tcp->getGlobalPose();
        Eigen::Matrix4f newPose = Eigen::Matrix4f::Identity();
        newPose.block(0, 0, 3, 3) = Kinematics::poseFromYAxis(currentPose.block(0, 1, 3, 1));
        newPose.block(0, 3, 3, 1) = immediateCapturePoint;
        ik->setGoal(newPose, tcp, VirtualRobot::IKSolver::All, 0.5f, 0.2f/180.0f*M_PI);
        ik->solveIK();

        std::cout << "Recovery IK error: Position: " << ik->getErrorPosition() << " Orientation: " << ik->getErrorRotation() << std::endl;

        nodes->getJointValues(trajectory);
        nodes->setJointValues(backup);

        return true;
    }

private:

    void updateCaputePoint(double dt)
    {
        // in mm
        const auto& com = colModelNodes->getCoM();
        velocityEstimator.update(com, dt);
        // in mm/s
        const auto& comVel = velocityEstimator.estimation;

        double gravity = 9.81 * 1000; // in mm/s^2
        immediateCapturePoint.head(2) = com.head(2) + comVel.head(2) * sqrt(com.z() / gravity);
    }

    double maxHullDist;

    VirtualRobot::RobotNodePtr leftFoot;
    VirtualRobot::RobotNodePtr rightFoot;
    VirtualRobot::MathTools::ConvexHull2DPtr leftSupportHull;
    VirtualRobot::MathTools::ConvexHull2DPtr rightSupportHull;

    VirtualRobot::RobotNodeSetPtr nodes;
    VirtualRobot::RobotNodeSetPtr colModelNodes;
    VirtualRobot::DifferentialIKPtr leftLegIK;
    VirtualRobot::DifferentialIKPtr rightLegIK;
    Bipedal::DerivationEstimator<Eigen::Vector3f> velocityEstimator;
    Eigen::Vector3f immediateCapturePoint;
    Eigen::Vector3f contactPoint;
};

#endif
