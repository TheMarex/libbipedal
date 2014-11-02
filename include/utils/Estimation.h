/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __VELOCITY_ESTIMATION_H__
#define __VELOCITY_ESTIMATION_H__

#include <Eigen/Dense>

#include <array>
#include <iostream>

/**
 * This implements estimating the derivative from a discrete input signal.
 *
 * The methode is based on finite differences outlined here:
 * https://en.wikipedia.org/wiki/Finite_difference_coefficient
 *
 * Beware: Only use higher order estimations if you know what you are doing.
 * In some cases it will cause a lot of noise (e.g. observed when trying to estimate the CoM acceleration)
 *
 */
namespace Bipedal
{
    using CoefficientTableT = std::array<double, 60>;

    constexpr CoefficientTableT DerivationCoefficientTable  = {
    // first derivative
    1,        -1,
    3/2.0,    -2, 1/2.0,
    11/6.0,   -3, 3/2.0,  -1/3.0,
    25/12.0,  -4, 3.0,    -4/3.0,   1/4.0,
    137/60.0, -5, 5.0,    -10/3.0,  5/4.0,  -1/5.0,
    49/20.0,  -6, 15/2.0, -20/3.0,  15/4.0, -6/5.0, 1/6.0,
    // second derivative
    -1,        2,        -1,
    -2,        5,        -4,        1,
    -35/12.0,  26/3.0,   -19/2.0,   14/3.0,   -11/12.0,
    -15/4.0,   77/6.0,   -107/6.0,  13,       -61/12.0, 5/6.0,
    -203/45.0, 87/5.0,   -117/4.0,  254/9.0,  -33/2.0,  27/5.0,   -137/180.0,
    -469/90.0, 223/10.0, -879/20.0, 949/18.0, -41,      201/10.0, -1019/180.0, 7/10.0,
    };
    constexpr std::array<unsigned, 12> DerivationCoefficientOffsets = {
    0,   2,  5,  9, 14, 20,
    27, 30, 34, 39, 45, 52,
    };

    /**
     * derivate is the order of the derivative and order the order of accuracy.
     */
    constexpr unsigned getCoefficientTableOffset(unsigned derivative, unsigned order)
    {
        return DerivationCoefficientOffsets[(derivative-1)*6 + (order-1)];
    }

    template<typename ValueT, unsigned ORDER, unsigned DERIVATIVE>
    class BackwardDerivationEstimator
    {
    public:
        ValueT estimation;
        ValueT zeroValue;
        unsigned initializationCounter;

        BackwardDerivationEstimator(ValueT zeroValue, ValueT initEstimation)
        : estimation(initEstimation)
        , zeroValue(zeroValue)
        , initializationCounter(0)
        {
            std::fill(prevValues.begin(), prevValues.end(), zeroValue);
        }

        void update(ValueT val, double dt)
        {
            // check that we do not devide by zero
            BOOST_ASSERT(std::fabs(dt) > std::numeric_limits<double>::epsilon());

            // shift array to right
            std::rotate(prevValues.rbegin(), prevValues.rbegin()+1, prevValues.rend());
            prevValues[0] = val;

            // use initial estimation until we have enough values
            if (initializationCounter >= prevValues.size())
            {
                estimation = zeroValue;
                unsigned offset = getCoefficientTableOffset(DERIVATIVE, ORDER);
                for (unsigned i = 0; i < prevValues.size(); i++)
                {
                    estimation += DerivationCoefficientTable[offset + i] * prevValues[i];
                }
                // FIXME We assume dt has ways the same value!
                estimation /= pow(dt, DERIVATIVE);
            }
            else
            {
                initializationCounter++;
            }
        }
    private:
        std::array<ValueT, ORDER+DERIVATIVE> prevValues;
    };

    inline Eigen::MatrixXf slopeEstimation(const Eigen::MatrixXf& trajectory, float timestep)
    {
        Eigen::MatrixXf slope;
        slope.resize(trajectory.rows(), trajectory.cols());
        slope.fill(0);

        Eigen::VectorXf start = trajectory.col(0);
        Eigen::VectorXf initialEstimation(start.rows());
        initialEstimation.fill(0);

        FirstDerivativeEstimator<Eigen::VectorXf> estimator(initialEstimation, initialEstimation);

        for (int i = 0; i < trajectory.cols() - 1; i++)
        {
            slope.col(i) = estimator.estimation;
            estimator.update(trajectory.col(i+1), timestep);
        }

        return slope;
    }

};

#endif

