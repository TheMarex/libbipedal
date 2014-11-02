/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __POSTURE_CONTROLLER_H__
#define __POSTURE_CONTROLLER_H__

#include <VirtualRobot/MathTools.h>
#include <Eigen/Dense>

#include "DampeningController.h"

namespace Bipedal
{

/**
 * Implements a dampened controller, that controlls the rotation around the x and y axis
 * of a reference frame.
 *
 * Based on the chest posture controller that was proposed by kajita in his 2010 paper.
 *
 * Note: The constant K in the paper is proportional, here it is anti-proportional
 * because I use the same dampening controller that is used for the foot forces/torques.
 *
 */
template<bool CORRECT_ROLL, bool CORRECT_PITCH, bool CORRECT_YAW>
struct PostureController
{
    PostureController(double rollFeedbackDampening=40,  double rollNeutralTime=5,
                      double pitchFeedbackDampening=80, double pitchNeutralTime=5,
                      double yawFeedbackDampening=80,   double yawNeutralTime=5)
    : phiDC(DampeningController {rollFeedbackDampening, rollNeutralTime, 0, 0})
    , thetaDC(DampeningController {pitchFeedbackDampening, pitchNeutralTime, 0, 0})
    , gammaDC(DampeningController {yawFeedbackDampening, yawNeutralTime, 0, 0})
    , refRPY(Eigen::Vector3f::Zero())
    , currentRPY(Eigen::Vector3f::Zero())
    {
    }

    double normAngle(double a) const
    {
        while (a > M_PI)
        {
            a -= 2*M_PI;
        }
        while (a < -M_PI)
        {
            a += 2*M_PI;
        }

        return a;
    }

    Eigen::Matrix4f correctPosture(const Eigen::Matrix4f& referencePosture,
                                   const Eigen::Matrix4f& currentPosture)
    {
        VirtualRobot::MathTools::eigen4f2rpy(referencePosture, refRPY);
        VirtualRobot::MathTools::eigen4f2rpy(currentPosture,   currentRPY);
        if (CORRECT_ROLL)
            phiDC.update(normAngle(refRPY.x() - currentRPY.x()));
        if (CORRECT_PITCH)
            thetaDC.update(normAngle(refRPY.y() - currentRPY.y()));
        if (CORRECT_YAW)
            gammaDC.update(normAngle(refRPY.z() - currentRPY.z()));
        Eigen::Matrix4f correctionMatrix = Eigen::Matrix4f::Identity();

        VirtualRobot::MathTools::rpy2eigen4f(phiDC.delta, thetaDC.delta, gammaDC.delta, correctionMatrix);

        return correctionMatrix;
    }

    Eigen::Vector3f refRPY;
    Eigen::Vector3f currentRPY;
    DampeningController phiDC;
    DampeningController thetaDC;
    DampeningController gammaDC;
};

}

#endif
