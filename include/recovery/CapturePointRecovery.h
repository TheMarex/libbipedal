#ifndef __CAPTURE_POINT_RECOVERY_H__
#define __CAPTURE_POINT_RECOVERY_H__

#include "../bipedal.h"
#include "PushRecovery.h"

#include "../utils/Estimation.h"
#include "../utils/Interpolation.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/MathTools.h>

class CapturePointRecovery : public PushRecovery
{
    enum RecoveryState
    {
        STATE_INITIAL   = 0,
        STATE_FALLING   = 1,
        STATE_RECOVERED = 2,
        STATE_FAILED    = 4,
    };

public:
    CapturePointRecovery(const VirtualRobot::RobotNodeSetPtr& colModelNodes,
                         const VirtualRobot::RobotNodePtr& leftFoot,
                         const VirtualRobot::RobotNodePtr& rightFoot,
                         const VirtualRobot::RobotNodePtr& chest,
                         const VirtualRobot::RobotNodePtr& pelvis,
                         const VirtualRobot::MathTools::ConvexHull2DPtr& leftSupportHull,
                         const VirtualRobot::MathTools::ConvexHull2DPtr& rightSupportHull);

    virtual const Eigen::Matrix4f& getLeftFootPose() const override;
    virtual const Eigen::Matrix4f& getRightFootPose() const override;
    virtual const Eigen::Matrix4f& getChestPose() const override;
    virtual const Eigen::Matrix4f& getPelvisPose() const override;
    virtual Kinematics::SupportPhase getSupportPhase() const override;
    virtual bool isFalling() const override;
    virtual void update(Kinematics::SupportPhase phase, double dt) override;

    const Eigen::Vector3f& getContactPoint() const;
    const Bipedal::CubicBezierCurve3f& getCaptureTrajectory() const;

    /**
     * Estimates the position of the iCP at time + t
     */
    Eigen::Vector3f getFutureCapturePoint(const VirtualRobot::RobotNodePtr& ankleNode, double t) const;

    /**
     * Get the current iCP
     */
    Eigen::Vector3f getImmediateCapturePoint() const;

private:
    void initializeRecovery(Kinematics::SupportPhase phase);

    void recomputeDualSupportHull();

    double maxHullDist;
    double minHeight;
    double minTime;
    unsigned minFallingFrames;
    unsigned fallingFrameCounter;
    Kinematics::SupportPhase lastSupportPhase;
    Kinematics::SupportPhase recoverySupportPhase;
    Bipedal::CubivBezierCurve3f recoveryTrajectory;
    RecoveryState state;

    Eigen::Matrix4f leftFootPose;
    Eigen::Matrix4f rightFootPose;
    Eigen::Matrix4f chestPose;
    Eigen::Matrix4f pelvisPose;

    VirtualRobot::RobotNodePtr leftFoot;
    VirtualRobot::RobotNodePtr rightFoot;
    VirtualRobot::RobotNodePtr chest;
    VirtualRobot::RobotNodePtr pelvis;
    VirtualRobot::MathTools::ConvexHull2DPtr leftSupportHull;
    VirtualRobot::MathTools::ConvexHull2DPtr rightSupportHull;
    VirtualRobot::MathTools::ConvexHull2DPtr dualSupportHull;

    VirtualRobot::RobotNodeSetPtr colModelNodes;
    Bipedal::DerivationEstimator<Eigen::Vector3f> velocityEstimator;
    Eigen::Vector3f contactPoint;
};

#endif
