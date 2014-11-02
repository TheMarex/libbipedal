/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __UTILS_ZMP_H__
#define __UTILS_ZMP_H__

#include <VirtualRobot/VirtualRobot.h>
#include "Estimation.h"

namespace Bipedal
{

/**
 * Computes ZMP according to the table cart model.
 *
 * If CoM is in mm, you need gravity in mm/s^2. Output is then in mm as well.
 */
inline Eigen::Vector2f computeModelZMP(const Eigen::Vector3f& com, const Eigen::Vector3f& comAcc, double gravity);

//! Note: We implicitly assume that the ZMP on a xy-plane at z = 0.
inline Eigen::Vector2f computeMultiBodyZMP(double mass,
                                           double gravity,
                                           const Eigen::Vector3f& com,
                                           const Eigen::Vector3f& linearMomentumDiff,
                                           const Eigen::Vector3f& angularMomentumDiff);

/**
 * Use Multi-Body methode to estimate the ZMP.
 */
class MultiBodyZMPEstimator
{
public:
    //! zmp in m
    Eigen::Vector2f estimation;

    /**
     * Mass in kg and gravity in m/s^2
     */
    MultiBodyZMPEstimator(double mass, double gravity);

    /**
     * CoM is in m.
     * Linear momentum is in N s
     * Angular momentum is in N m s.
     * dt in s
     */
    void update(const Eigen::Vector3f& com,
                const Eigen::Vector3f& linearMomentum,
                const Eigen::Vector3f& angularMomentum,
                double dt);

private:
    double mass;
    double gravity;
    FirstDerivativeEstimator<Eigen::Vector3f> linearMomentumDiff;
    FirstDerivativeEstimator<Eigen::Vector3f> angularMomentumDiff;
};

/**
 * Use Cart-Table model to predict the ZMP.
 */
class CartTableZMPEstimator
{
public:
    //! zmp in m
    Eigen::Vector2f estimation;

    //! Gravity in m/s^2
    CartTableZMPEstimator(double gravity);

    //! CoM in m, CoM velocity in m/s, dt in s.
    void update(const Eigen::Vector3f& com, const Eigen::Vector3f& comVel, double dt);

private:
    double gravity;
    FirstDerivativeEstimator<Eigen::Vector3f> accelerationEstimator;
};

/**
 * Estimates the ZMP based on the CoP.
 */
class CoPZMPEstimator
{
public:
    //! zmp in m
    Eigen::Vector2f estimation;

    CoPZMPEstimator(const VirtualRobot::ContactSensorPtr& leftFootSensor,
                    const VirtualRobot::ContactSensorPtr& rightFootSensor);

    void update(float dt);

private:
    VirtualRobot::ContactSensorPtr leftFootSensor;
    VirtualRobot::ContactSensorPtr rightFootSensor;
};

}

#endif
