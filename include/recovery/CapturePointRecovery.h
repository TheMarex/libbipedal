#ifndef __CAPTURE_POINT_RECOVERY_H__
#define __CAPTURE_POINT_RECOVERY_H__

#include "../bipedal.h"
#include "PushRecovery.h"

#include "../utils/Estimation.h"
#include "../utils/Interpolation.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/MathTools.h>

namespace Bipedal
{

class CapturePointRecovery : public PushRecovery
{
public:
    CapturePointRecovery(const VirtualRobot::RobotNodeSetPtr& colModelNodes,
                         const VirtualRobot::RobotNodePtr& leftFoot,
                         const VirtualRobot::RobotNodePtr& rightFoot,
                         const VirtualRobot::RobotNodePtr& chest,
                         const VirtualRobot::RobotNodePtr& pelvis);

    virtual void update(double dt) override;
    virtual void startRecovering(Bipedal::SupportPhase currentPhase) override;
    virtual bool isRecovering() const override;

    virtual const Eigen::Matrix4f& getLeftFootPose() const override;
    virtual const Eigen::Matrix4f& getRightFootPose() const override;
    virtual const Eigen::Matrix4f& getChestPose() const override;
    virtual const Eigen::Matrix4f& getPelvisPose() const override;

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
    double minHeight;
    double minTime;
    Bipedal::SupportPhase recoverySupportPhase;
    Bipedal::CubivBezierCurve3f recoveryTrajectory;

    Eigen::Matrix4f leftFootPose;
    Eigen::Matrix4f rightFootPose;
    Eigen::Matrix4f chestPose;
    Eigen::Matrix4f pelvisPose;
    Eigen::Matrix4f initialLeftFootPose;
    Eigen::Matrix4f initialRightFootPose;
    Eigen::Matrix4f initialChestPose;
    Eigen::Matrix4f initialPelvisPose;

    VirtualRobot::RobotNodePtr leftFoot;
    VirtualRobot::RobotNodePtr rightFoot;
    VirtualRobot::RobotNodePtr chest;
    VirtualRobot::RobotNodePtr pelvis;

    VirtualRobot::RobotNodeSetPtr colModelNodes;
    Bipedal::DerivationEstimator<Eigen::Vector3f> velocityEstimator;
};

}

#endif
