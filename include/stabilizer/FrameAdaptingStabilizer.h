/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __FRAME_ADAPTING_STABILIZER_H_
#define __FRAME_ADAPTING_STABILIZER_H_

#include <Eigen/Dense>

#include <unordered_map>

namespace Bipedal
{

class DampeningController;

class FrameAdaptingStabilizer
{
public:
    virtual const Eigen::Matrix4f& getChestPoseRef() = 0;
    virtual const Eigen::Matrix4f& getPelvisPoseRef() = 0;
    virtual const Eigen::Matrix4f& getLeftFootPoseRef() = 0;
    virtual const Eigen::Matrix4f& getRightFootPoseRef() = 0;
    virtual const Eigen::Vector3f& getCoMPositionRef() = 0;
    virtual const Eigen::Vector3f& getZMPPositionRef() = 0;

    virtual const Eigen::Matrix4f& getChestPose() = 0;
    virtual const Eigen::Matrix4f& getPelvisPose() = 0;
    virtual const Eigen::Matrix4f& getLeftFootPose() = 0;
    virtual const Eigen::Matrix4f& getRightFootPose() = 0;
    virtual const Eigen::Vector3f& getCoMPosition() = 0;
    virtual const Eigen::Vector3f& getZMPPosition() = 0;

    virtual std::unordered_map<std::string, DampeningController*> getControllers() = 0;

    virtual void update(float dt,
                        const Eigen::Vector3f& comPosition,
                        const Eigen::Vector3f& comVelocity,
                        const Eigen::Vector3f& zmpPosition,
                        Bipedal::SupportPhase phase,
                        const Eigen::Matrix4f& chestPoseRef,
                        const Eigen::Matrix4f& pelvisPoseRef,
                        const Eigen::Matrix4f& leftFootPoseRef,
                        const Eigen::Matrix4f& rightFootPoseRef,
                        const Eigen::Vector3f& comPositionRef,
                        const Eigen::Vector3f& comVelocityRef,
                        const Eigen::Vector3f& zmpPositionRef) = 0;
};

}

#endif
