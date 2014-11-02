/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __PUSH_RECOVERY_H__
#define __PUSH_RECOVERY_H__

#include "../bipedal.h"

#include <Eigen/Dense>

namespace Bipedal
{

/**
 * This class is an interface to push recovery mechanism.
 */
class PushRecovery
{
public:
    /**
     * This needs to be called continiously to update the internal state to track
     * the CoM movement.
     */
    virtual void update(const Eigen::Vector3f& com, const Eigen::Vector3f& comVel, double dt) = 0;

    /**
     * Computes the trajectory to recover from the current state to a full stop.
     * After each call to update() the target poses can be extracted by the getters
     * below.
     */
    virtual void startRecovering(Bipedal::SupportPhase currentPhase) = 0;
    virtual bool isRecovering() const = 0;

    virtual const Eigen::Matrix4f& getLeftFootPose() const = 0;
    virtual const Eigen::Matrix4f& getRightFootPose() const = 0;
    virtual const Eigen::Matrix4f& getChestPose() const = 0;
    virtual const Eigen::Matrix4f& getPelvisPose() const = 0;
    virtual Bipedal::SupportPhase getSupportPhase() const = 0;
};

}

#endif
