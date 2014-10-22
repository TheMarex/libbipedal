#ifndef __FOOT_FORCE_CONTROLLER_H__
#define __FOOT_FORCE_CONTROLLER_H__

#include <VirtualRobot/MathTools.h>
#include <Eigen/Dense>

#include <boost/assert.hpp>

#include <bipedal/controller/DampeningController.h>

namespace Bipedal
{

/**
 * Controlls the force difference in booth feet using the rotation of the pelvis link.
 */
class FootForceController
{
public:
    FootForceController(double hipJointDistance)
    : hipJointDistance(hipJointDistance)
    , zCtrlDC(DampeningController {3000.0, 10.0, 0.0, 0.0})
    {
        BOOST_ASSERT(hipJointDistance > 0);
    }

    /**
     * For robots that can compensate a tilt pelvis.
     */
    Eigen::Matrix4f correctPelvisOrientation(const Eigen::Matrix4f& orientationRef,
                                             const Eigen::Vector3f& leftFootForceRef,
                                             const Eigen::Vector3f& rightFootForceRef,
                                             const Eigen::Vector3f& leftFootForce,
                                             const Eigen::Vector3f& rightFootForce)
    {
        zCtrlDC.update((leftFootForceRef.z() - rightFootForceRef.z()) - (leftFootForce.z() - rightFootForce.z()));

        Eigen::Matrix4f correctionMatrix = Eigen::Matrix4f::Zero();
        VirtualRobot::MathTools::rpy2eigen4f(0, zCtrlDC.delta / hipJointDistance, 0, correctionMatrix);
        return orientationRef * correctionMatrix;
    }

    void correctFootHeight(const Eigen::Vector3f& leftFootForceRef,
                           const Eigen::Vector3f& rightFootForceRef,
                           const Eigen::Vector3f& leftFootForce,
                           const Eigen::Vector3f& rightFootForce,
                           Eigen::Matrix4f& leftFootPose,
                           Eigen::Matrix4f& rightFootPose)
    {
        zCtrlDC.update((leftFootForceRef.z() - rightFootForceRef.z()) - (leftFootForce.z() - rightFootForce.z()));

        leftFootPose(2, 3)  = leftFootPose(2, 3) - zCtrlDC.delta / 2.0;
        rightFootPose(2, 3) = rightFootPose(2, 3) + zCtrlDC.delta / 2.0;

    }

public:
    double hipJointDistance;
    DampeningController zCtrlDC;
};

}

#endif
