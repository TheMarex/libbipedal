#ifndef __UTILS_ZMP_H__
#define __UTILS_ZMP_H__

#include "Estimation.h"

namespace Bipedal
{
    /*
     * Computes ZMP according to the table cart model.
     */
    inline Eigen::Vector2f computeModelZMP(Eigen::Vector3f com, Eigen::Vector3f comAcc, double gravity)
    {
        Eigen::Vector2f zmp;
        // note that gravity.z() < 0, so we changed the sign of the term
        zmp.x() = com.x() + com.z() / gravity * comAcc.x();
        zmp.y() = com.y() + com.z() / gravity * comAcc.y();
        return zmp;
    }

    /**
     * Note: We implicitly assume that the ZMP on a xy-plane at z = 0.
     */
    inline Eigen::Vector2f computeMultiBodyZMP(double mass,
                                               double gravity,
                                               const Eigen::Vector3f& com,
                                               const Eigen::Vector3f& linearMomentumDiff,
                                               const Eigen::Vector3f& angularMomentumDiff)
    {
        Eigen::Vector2f zmp;
        double norm = mass * gravity + linearMomentumDiff.z();
        zmp.x() = mass * gravity * com.x() - angularMomentumDiff.y();
        zmp.y() = mass * gravity * com.y() - angularMomentumDiff.x();
        zmp /= norm;

        return zmp;
    }

    class MultiBodyZMPEstimator
    {
    public:
        Eigen::Vector2f estimation;

        MultiBodyZMPEstimator(double mass, double gravity)
        : mass(mass)
        , gravity(gravity)
        , estimation(Eigen::Vector2f::Zero())
        , linearMomentumDiff(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero())
        , angularMomentumDiff(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero())
        {
        }

        void update(const Eigen::Vector3f& com,
                    const Eigen::Vector3f& linearMomentum,
                    const Eigen::Vector3f& angularMomentum,
                    double dt)
        {
            linearMomentumDiff.update(linearMomentum, dt);
            angularMomentumDiff.update(angularMomentum, dt);

            estimation = computeMultiBodyZMP(mass, gravity, com, linearMomentumDiff.estimation, angularMomentumDiff.estimation);
        }

    private:
        double mass;
        double gravity;
        ThirdOrderBackwardDerivationEstimator<Eigen::Vector3f> linearMomentumDiff;
        ThirdOrderBackwardDerivationEstimator<Eigen::Vector3f> angularMomentumDiff;
    };

    class CartTableZMPEstimator
    {
    public:
        Eigen::Vector2f estimation;

        CartTableZMPEstimator(double gravity)
        : gravity(gravity)
        , accelerationEstimator(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero())
        {
        }

        void update(const Eigen::Vector3f& com, double dt)
        {
            estimation = computeModelZMP(com, accelerationEstimator.estimation, gravity);
        }

    private:
        double gravity;
        SecondDerivativeEstimator<Eigen::Vector3f> accelerationEstimator;
    };
};

#endif
