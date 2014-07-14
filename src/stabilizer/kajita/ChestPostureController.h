#ifndef __CHEST_POSTURE_CONTROLLER_H__
#define __CHEST_POSTURE_CONTROLLER_H__

#include <VirtualRobot/MathTools.h>
#include <Eigen/Dense>

#include "DampeningController.h"

/**
 * Implements the chest posture controller proposed by Kajita in his 2010 paper.
 *
 * Note: The constant K in the paper is proportional, here it is anti-proportional
 * because I use the same dampening controller that is used for the foot forces/torques.
 *
 */
struct ChestPostureController
{
    ChestPostureController()
    : phiDC(DampeningController {4, 5.0, 0})
    , thetaDC(DampeningController {20, 5.0, 0})
    , refRPY(Eigen::Vector3f::Zero())
    , currentRPY(Eigen::Vector3f::Zero())
    {
    }

    Eigen::Matrix4f correctPosture(const Eigen::Matrix4f& referencePosture,
                                   const Eigen::Matrix4f& currentPosture)
    {
        VirtualRobot::MathTools::eigen4f2rpy(referencePosture, refRPY);
        VirtualRobot::MathTools::eigen4f2rpy(currentPosture,   currentRPY);
        phiDC.update(refRPY.x() - currentRPY.x());
        thetaDC.update(refRPY.y() - currentRPY.y());
        Eigen::Matrix4f correctionMatrix;

        VirtualRobot::MathTools::rpy2eigen4f(phiDC.delta, thetaDC.delta, 0, correctionMatrix);

        return referencePosture * correctionMatrix;
    }

    Eigen::Vector3f refRPY;
    Eigen::Vector3f currentRPY;
    DampeningController phiDC;
    DampeningController thetaDC;
};

#endif
