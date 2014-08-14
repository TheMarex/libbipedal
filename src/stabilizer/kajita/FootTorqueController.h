#ifndef __FOOT_TORQUE_CONTROLLER_H__
#define __FOOT_TORQUE_CONTROLLER_H__

#include <VirtualRobot/MathTools.h>
#include <Eigen/Dense>

#include "utils/Kinematics.h"

#include "DampeningController.h"

class FootTorqueController
{
public:
    // FIXME values are only guessed
    FootTorqueController()
    : leftPhiDC(DampeningController    {100000, 2.0, 0.0})
    , leftThetaDC(DampeningController  {100000, 2.0, 0.0})
    , rightPhiDC(DampeningController   {100000, 2.0, 0.0})
    , rightThetaDC(DampeningController {100000, 2.0, 0.0})
    {
    }

    void correctFootOrientation(const Eigen::Matrix4f& leftOrientationRef,
                                const Eigen::Matrix4f& rightOrientationRef,
                                const Eigen::Vector3f& leftTorqueRef,
                                const Eigen::Vector3f& rightTorqueRef,
                                const Eigen::Vector3f& leftTorque,
                                const Eigen::Vector3f& rightTorque,
                                Eigen::Matrix4f& leftOrientation,
                                Eigen::Matrix4f& rightOrientation)
    {
        leftOrientation  = leftOrientationRef  * computeCorrectionMatrix(leftTorqueRef,
                                                                         leftTorque,
                                                                         leftPhiDC,
                                                                         leftThetaDC);
        rightOrientation = rightOrientationRef * computeCorrectionMatrix(rightTorqueRef,
                                                                         rightTorque,
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

    DampeningController leftPhiDC;
    DampeningController leftThetaDC;
    DampeningController rightPhiDC;
    DampeningController rightThetaDC;
};

#endif
