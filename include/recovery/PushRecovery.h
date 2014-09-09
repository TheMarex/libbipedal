#ifndef __PUSH_RECOVERY_H__
#define __PUSH_RECOVERY_H__

#include <Eigen/Dense>

/**
 * This class is an interface to push recovery mechanism.
 */
class PushRecovery
{
public:
    /**
     * dt is the time in seconds since the last call to isFalling
     */
    virtual bool isFalling(Kinematics::SupportPhase phase, double dt) = 0;

    /**
     * Returns angles that need to be realized to recover from fall.
     */
    virtual bool computeRecoveryTrajectory(Kinematics::SupportPhase phase, Eigen::VectorXf& targetAngles) = 0;
};

#endif
