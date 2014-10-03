#ifndef __UTILS_ZMP_H__
#define __UTILS_ZMP_H__

#include "Estimation.h"

namespace Bipedal
{
    /*
     * Computes ZMP according to the table cart model.
     *
     * If CoM is in mm, you need gravity in mm/s^2. Output is then in mm as well.
     */
    inline Eigen::Vector2f computeModelZMP(const Eigen::Vector3f& com, const Eigen::Vector3f& comAcc, double gravity)
    {
        Eigen::Vector2f zmp;
        zmp.x() = com.x() - com.z() / gravity * comAcc.x();
        zmp.y() = com.y() - com.z() / gravity * comAcc.y();
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
        zmp.y() = mass * gravity * com.y() + angularMomentumDiff.x();
        zmp /= norm;

        return zmp;
    }

    class MultiBodyZMPEstimator
    {
    public:
        Eigen::Vector2f estimation;

        /**
         * Mass in kg and gravity in m/s^2
         */
        MultiBodyZMPEstimator(double mass, double gravity)
        : mass(mass)
        , gravity(gravity)
        , estimation(Eigen::Vector2f::Zero())
        , linearMomentumDiff(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero())
        , angularMomentumDiff(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero())
        {
            BOOST_ASSERT(gravity > 0);
        }

        /**
         * CoM is in m.
         * Linear momentum is in N s
         * Angular momentum is in N m s.
         * dt in s
         */
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
        FirstDerivativeEstimator<Eigen::Vector3f> linearMomentumDiff;
        FirstDerivativeEstimator<Eigen::Vector3f> angularMomentumDiff;
    };

    class CartTableZMPEstimator
    {
    public:
        Eigen::Vector2f estimation;

        /**
         * Gravity in m/s^2
         */
        CartTableZMPEstimator(double gravity)
        : gravity(gravity)
        , accelerationEstimator(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero())
        {
            BOOST_ASSERT(gravity > 0);
        }

        /**
         * CoM in m, CoM velocity in m/s, dt in s.
         */
        void update(const Eigen::Vector3f& com, const Eigen::Vector3f& comVel, double dt)
        {
            accelerationEstimator.update(comVel, dt);
            estimation = computeModelZMP(com, accelerationEstimator.estimation, gravity);
        }

    private:
        double gravity;
        FirstDerivativeEstimator<Eigen::Vector3f> accelerationEstimator;
    };
};

#endif
