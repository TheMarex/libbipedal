#ifndef __VELOCITY_ESTIMATION_H__
#define __VELOCITY_ESTIMATION_H__

#include <Eigen/Dense>

#include <array>
#include <iostream>

namespace Bipedal
{
    constexpr std::array<double, 36> DerivationCoefficientTable  = {
    1,        -1,
    3/2.0,    -2, 1/2.0,
    11/6.0,   -3, 3/2.0,  -1/3.0,
    25/12.0,  -4, 3.0,    -4/3.0,   1/4.0,
    137/60.0, -5, 5.0,    -10/3.0,  5/4.0,  -1/5.0,
    49/20.0,  -6, 15/2.0, -20/3.0,  15/4.0, -6/5.0, 1/6.0
    };

    constexpr unsigned getCoefficientTableOffset(unsigned order)
    {
        return order <= 1 ? 0 : (order + getCoefficientTableOffset(order-1));
    }

    template<typename ValueT, unsigned ORDER>
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
            if (initializationCounter >= ORDER)
            {
                estimation = zeroValue;
                unsigned offset = getCoefficientTableOffset(ORDER);
                for (unsigned i = 0; i < prevValues.size(); i++)
                {
                    estimation += DerivationCoefficientTable[offset + i] * prevValues[i];
                }
                // FIXME We assume dt has ways the same value!
                estimation /= dt;
            }
            else
            {
                initializationCounter++;
            }
        }
    private:
        std::array<ValueT, ORDER+1> prevValues;
    };

    inline Eigen::MatrixXf slopeEstimation(const Eigen::MatrixXf& trajectory, float timestep)
    {
        Eigen::MatrixXf slope;
        slope.resize(trajectory.rows(), trajectory.cols());
        slope.fill(0);

        Eigen::VectorXf start = trajectory.col(0);
        Eigen::VectorXf initialEstimation(start.rows());
        initialEstimation.fill(0);

        DerivationEstimator<Eigen::VectorXf> estimator(initialEstimation, initialEstimation);

        for (int i = 0; i < trajectory.cols() - 1; i++)
        {
            slope.col(i) = estimator.estimation;
            estimator.update(trajectory.col(i+1), timestep);
        }

        return slope;
    }

};

#endif

