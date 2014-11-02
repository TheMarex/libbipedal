/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __FALL_DETECTOR_H__
#define __FALL_DETECTOR_H__

#include "../bipedal.h"

namespace Bipedal
{

/**
 * This class is an interface to detect instability for a given trajectory
 */
class FallDetector
{
public:
    /**
     * dt is the time in seconds since the last call to this function
     * Needs to be called in each iteration of the control loop before
     * the call to isFalling.
     */
    virtual void update(const Eigen::Vector3f& com,
                        const Eigen::Vector3f& comVel,
                        const Eigen::Vector3f& linearMomentum,
                        const Eigen::Vector3f& angularMomentum,
                        double dt) = 0;

    /**
     * Returns true if it was detected that we are falling.
     */
    virtual bool isFalling() const = 0;
};

}

#endif
