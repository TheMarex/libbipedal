#ifndef _ZMP_TRACKING_CONTROLLER_H_
#define _ZMP_TRACKING_CONTROLLER_H_

#include <Eigen/Dense>

namespace Bipedal
{

/**
 * Adapts the ZMP reference to follow the real state.
 */
class ZMPTrackingController
{
public:
    ZMPTrackingController(Eigen::Vector3f gain)
    : feedbackGain(gain)
    {
    }

    Eigen::Vector2f computeAdaptedZMP(const Eigen::Vector2f& refZMP, const Eigen::Vector2f& refCoM, const Eigen::Vector2f& refCoMVel,
                                      const Eigen::Vector2f& zmp, const Eigen::Vector2f& com, const Eigen::Vector2f& comVel)
    {
        Eigen::Vector3f refStateX(refCoM.x(), refCoMVel.x(), refZMP.x());
        Eigen::Vector3f refStateY(refCoM.y(), refCoMVel.y(), refZMP.y());
        Eigen::Vector3f stateX(com.x(), comVel.x(), zmp.x());
        Eigen::Vector3f stateY(com.y(), comVel.y(), zmp.y());

        Eigen::Vector2f adaptedZMP;
        adaptedZMP.x() = feedbackGain.dot(refStateX - stateX) + refZMP.x();
        adaptedZMP.y() = feedbackGain.dot(refStateY - stateY) + refZMP.y();

        return adaptedZMP;
    }

private:
    Eigen::Vector3f feedbackGain;
};

}

#endif
