/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
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
    CapturePointRecovery(const VirtualRobot::RobotNodePtr& leftFoot,
                         const VirtualRobot::RobotNodePtr& rightFoot,
                         const VirtualRobot::RobotNodePtr& chest,
                         const VirtualRobot::RobotNodePtr& pelvis);

    virtual void update(const Eigen::Vector3f& com, const Eigen::Vector3f& comVel, double dt) override;
    virtual void startRecovering(Bipedal::SupportPhase currentPhase) override;
    virtual bool isRecovering() const override;

    virtual const Eigen::Matrix4f& getLeftFootPose() const override;
    virtual const Eigen::Matrix4f& getRightFootPose() const override;
    virtual const Eigen::Matrix4f& getChestPose() const override;
    virtual const Eigen::Matrix4f& getPelvisPose() const override;
    virtual Bipedal::SupportPhase getSupportPhase() const override;

    const Bipedal::CubicBezierCurve3f& getCaptureTrajectory() const;

private:
    bool recovering;
    double minHeight;
    double minTime;
    double gravity;
    Bipedal::SupportPhase recoverySupportPhase;
    Bipedal::CubivBezierCurve3f recoveryTrajectory;
    Eigen::Vector3f comPosition;
    Eigen::Vector3f comVelocity;

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
};

}

#endif
