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
    enum RecoveryState
    {
        STATE_RECOVERED  = 0,
        STATE_RAISE_FOOT = 1,
        STATE_LOWER_FOOT = 2,
    };

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
    , maxHullDist(100)
    , minHeight(100)
    , lastSupportPhase(Kinematics::SUPPORT_NONE)
    , contactPoint(Eigen::Vector3f::Zero())
    , state(STATE_RECOVERED)
    {
    }

    const Eigen::Vector3f& getCapturePoint() const { return immediateCapturePoint; }
    const Eigen::Vector3f& getContactPoint() const { return contactPoint; }

    virtual bool isFalling(Kinematics::SupportPhase phase, double dt) override
    {
        updateCaputePoint(dt);

        const auto& supportHull = [this, phase]() {
            if (phase == Kinematics::SUPPORT_LEFT)
                return leftSupportHull;
            if (phase == Kinematics::SUPPORT_RIGHT)
                return rightSupportHull;

            BOOST_ASSERT(phase == Kinematics::SUPPORT_BOTH);

            // Foot positions changed
            if (lastSupportPhase != phase)
            {
                recomputeDualSupportHull();
            }

            return dualSupportHull;
        }();
        lastSupportPhase = phase;
        // center of convex hull and orientation should correspond with ground frame
        const auto& groundFrame = Kinematics::computeGroundFrame(leftFoot->getGlobalPose(), rightFoot->getGlobalPose(), phase);

        Eigen::Vector3f iCPConvexHull = VirtualRobot::MathTools::transformPosition(immediateCapturePoint, groundFrame.inverse());

        // if CP is inside of the CH, we are not falling
        if (VirtualRobot::MathTools::isInside(iCPConvexHull.head(2), supportHull))
        {
            return false;
        }

        Eigen::Vector3f contact = Eigen::Vector3f::Zero();
        contact.head(2) = Walking::computeHullContactPoint(iCPConvexHull.head(2), supportHull);

        contactPoint = VirtualRobot::MathTools::transformPosition(contact, groundFrame);

        double dist = (contact-iCPConvexHull).norm();
        bool falling = dist > maxHullDist;
        if (falling && state == STATE_RECOVERED)
        {
            state = STATE_RAISE_FOOT;
        }

        return falling;
    }

    virtual bool computeRecoveryTrajectory(Kinematics::SupportPhase phase, Eigen::VectorXf& trajectory) override
    {
        if (state == STATE_RECOVERED)
        {
            return true;
        }

        // chose foot that is closest to CP
        const double distRight = (immediateCapturePoint - rightFoot->getGlobalPose().block(0, 3, 3, 1)).norm();
        const double distLeft  = (immediateCapturePoint - leftFoot->getGlobalPose().block(0, 3, 3, 1)).norm();

        const auto& tcp = distRight < distLeft ? rightFoot  : leftFoot;
        const auto& ik  = distRight < distLeft ? rightLegIK : leftLegIK;

        const auto& currentPose = tcp->getGlobalPose();
        Eigen::Vector3f zOffset = Eigen::Vector3f::Zero();

        const double height = currentPose(2, 3);

        if (state == STATE_RAISE_FOOT)
        {
            if (height >= minHeight)
            {
                state = STATE_LOWER_FOOT;
            }
            else
            {
                // multiplicator just makes sure we get there faster
                zOffset.z() = minHeight*2;
            }
        }
        else if (std::abs(height) < 0.5)
        {
            std::cout << "Recovered!" << std::endl;
            state = STATE_RECOVERED;
            return true;
        }

        Eigen::VectorXf backup;
        nodes->getJointValues(backup);
        Eigen::Matrix4f newPose = Eigen::Matrix4f::Identity();
        newPose.block(0, 0, 3, 3) = Kinematics::poseFromYAxis(currentPose.block(0, 1, 3, 1));
        newPose.block(0, 3, 3, 1) = immediateCapturePoint + zOffset;
        ik->setGoal(newPose, tcp, VirtualRobot::IKSolver::All, 0.5f, 0.2f/180.0f*M_PI);
        ik->solveIK();

        std::cout << "Recovery IK error: Position: " << ik->getErrorPosition() << " Orientation: " << ik->getErrorRotation() << std::endl;

        nodes->getJointValues(trajectory);
        nodes->setJointValues(backup);

        return false;
    }

private:

    void recomputeDualSupportHull()
    {
        Eigen::Vector2f offset = rightFoot->getGlobalPose().block(0, 3, 2, 1) - leftFoot->getGlobalPose().block(0, 3, 2, 1);

        std::vector<Eigen::Vector2f> points;
        for (const auto& v : leftSupportHull->vertices)
        {
            points.push_back(v - offset/2);
        }
        for (const auto& v : rightSupportHull->vertices)
        {
            points.push_back(v + offset/2);
        }

        dualSupportHull = VirtualRobot::MathTools::createConvexHull2D(points);
        Walking::CenterConvexHull(dualSupportHull);
    }

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
    double minHeight;
    Kinematics::SupportPhase lastSupportPhase;
    RecoveryState state;

    VirtualRobot::RobotNodePtr leftFoot;
    VirtualRobot::RobotNodePtr rightFoot;
    VirtualRobot::MathTools::ConvexHull2DPtr leftSupportHull;
    VirtualRobot::MathTools::ConvexHull2DPtr rightSupportHull;
    VirtualRobot::MathTools::ConvexHull2DPtr dualSupportHull;

    VirtualRobot::RobotNodeSetPtr nodes;
    VirtualRobot::RobotNodeSetPtr colModelNodes;
    VirtualRobot::DifferentialIKPtr leftLegIK;
    VirtualRobot::DifferentialIKPtr rightLegIK;
    Bipedal::DerivationEstimator<Eigen::Vector3f> velocityEstimator;
    Eigen::Vector3f immediateCapturePoint;
    Eigen::Vector3f contactPoint;
};

#endif
