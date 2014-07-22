#ifndef __VELOCITY_ESTIMATION_H__
#define __VELOCITY_ESTIMATION_H__

#include <Eigen/Dense>

namespace Bipedal
{
    inline Eigen::MatrixXf simpleDiffVelocityEstimation(const Eigen::MatrixXf& trajectory, float timestep)
    {
        Eigen::MatrixXf velocity;
        velocity.resize(trajectory.rows(), trajectory.cols());
        velocity.fill(0);

        for (int i = 0; i < trajectory.cols() - 1; i++)
        {
            velocity.col(i) = (trajectory.col(i + 1) - trajectory.col(i)) / timestep;
        }

        return velocity;
    }

    template<typename ValueT>
    struct DerivationEstimator
    {
        ValueT lastVal;
        ValueT estimation;
        DerivationEstimator(ValueT initLastVal, ValueT initEstimation)
        : lastVal(initLastVal)
        , estimation(initEstimation)
        {
        }

        void update(ValueT val, double dt)
        {
            if (std::fabs(dt) < std::numeric_limits<double>::epsilon())
            {
                lastVal = val;
                return;
            }

            estimation = (val - lastVal) / dt;
            lastVal = val;
        }
    };

};

#endif

