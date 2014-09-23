#ifndef __PUSH_RECOVERY_H__
#define __PUSH_RECOVERY_H__

#include "../bipedal.h"

#include <Eigen/Dense>

/**
 * This class is an interface to push recovery mechanism.
 */
class PushRecovery
{
public:
    /**
     * dt is the time in seconds since the last call to this function
     * Needs to be called in each iteration of the control loop before
     * the call to isFalling.
     */
    virtual void update(Kinematics::SupportPhase phase, double dt) = 0;

    /**
     * Returns true if it was detected that we are falling.
     */
    virtual bool isFalling() const = 0;

    virtual const Eigen::Matrix4f& getLeftFootPose() const = 0;
    virtual const Eigen::Matrix4f& getRightFootPose() const = 0;
    virtual const Eigen::Matrix4f& getChestPose() const = 0;
    virtual const Eigen::Matrix4f& getPelvisPose() const = 0;
    virtual Kinematics::SupportPhase getSupportPhase() const = 0;
};

#endif
