#ifndef __FOOT_TORQUE_CONTROLLER_H__
#define __FOOT_TORQUE_CONTROLLER_H__

#include <VirtualRobot/MathTools.h>
#include <Eigen/Dense>

#include "utils/Kinematics.h"

#include <bipedal/controller/DampeningController.h>

class FootTorqueController
{
public:
    // FIXME values are only guessed
    FootTorqueController()
    : leftPhiDC(DampeningController    {900000, 20.0, 0.0, 0.0})
    , leftThetaDC(DampeningController  {900000, 20.0, 0.0, 0.0})
    , rightPhiDC(DampeningController   {900000, 20.0, 0.0, 0.0})
    , rightThetaDC(DampeningController {900000, 20.0, 0.0, 0.0})
    {
    }

    void correctFootOrientation(const Eigen::Matrix4f& leftOrientationRef,
                                const Eigen::Matrix4f& rightOrientationRef,
                                const Eigen::Vector3f& leftTorqueRefLocal,
                                const Eigen::Vector3f& rightTorqueRefLocal,
                                const Eigen::Vector3f& leftTorqueLocal,
                                const Eigen::Vector3f& rightTorqueLocal,
                                Eigen::Matrix4f& leftOrientation,
                                Eigen::Matrix4f& rightOrientation)
    {
        leftOrientation  = leftOrientationRef  * computeCorrectionMatrix(leftTorqueRefLocal,
                                                                         leftTorqueLocal,
                                                                         leftPhiDC,
                                                                         leftThetaDC);
        rightOrientation = rightOrientationRef * computeCorrectionMatrix(rightTorqueRefLocal,
                                                                         rightTorqueLocal,
                                                                         rightPhiDC,
                                                                         rightThetaDC);
    }
private:
    Eigen::Matrix4f computeCorrectionMatrix(const Eigen::Vector3f& torqueRef,
                                            const Eigen::Vector3f& torque,
                                            DampeningController& phiDC,
                                            DampeningController& thetaDC)
    {
        phiDC.update(torqueRef.x() - torque.x());
        thetaDC.update(torqueRef.y() - torque.y());

        Eigen::Matrix4f correctionMatrix;
        VirtualRobot::MathTools::rpy2eigen4f(phiDC.delta, thetaDC.delta, 0, correctionMatrix);
        return correctionMatrix;
    }

public:
    DampeningController leftPhiDC;
    DampeningController leftThetaDC;
    DampeningController rightPhiDC;
    DampeningController rightThetaDC;
};

#endif
