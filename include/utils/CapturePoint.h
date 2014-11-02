/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __CAPTURE_POINT_H__
#define __CAPTURE_POINT_H__

#include <Eigen/Dense>

#include "../bipedal.h"

namespace Bipedal {

/**
 * Compute the current immediate capture point.
 *
 * com in m
 * comVel in m/s
 *
 * Returns iCP in m
 */
Eigen::Vector3f computeImmediateCapturePoint(const Eigen::Vector3f& com,
                                             const Eigen::Vector3f& comVel,
                                             double gravity)
{
    BOOST_ASSERT(gravity > 0);

    Eigen::Vector3f iCP = Eigen::Vector3f::Zero();
    iCP.head(2) = com.head(2) + comVel.head(2) * sqrt(com.z() / gravity);

    return iCP;
}

/**
 * Estimates the position of the iCP at time + t
 *
 * com in m
 * comVel in m/s
 * anklePosition in m
 * g in m/s^2
 *
 * Returns future capture point in m.
 */
Eigen::Vector3f computeFutureCapturePoint(const Eigen::Vector3f& com,
                                          const Eigen::Vector3f& comVel,
                                          const Eigen::Vector3f& anklePosition,
                                          double gravity, double t)
{
    BOOST_ASSERT(gravity > 0);

    double omega0 = sqrt(gravity / com.z());

    Eigen::Vector3f iCP = computeImmediateCapturePoint(com, comVel, gravity);
    Eigen::Vector3f ankle(anklePosition.x(), anklePosition.y(), 0);

    Eigen::Vector3f futureCP = (iCP -  ankle) * exp(t * omega0) + ankle;

    return futureCP;
}

}

#endif
