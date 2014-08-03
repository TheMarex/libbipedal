#ifndef __VELOCITY_ESTIMATION_H__
#define __VELOCITY_ESTIMATION_H__

#include <Eigen/Dense>

namespace Bipedal
{
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

    inline Eigen::MatrixXf slopeEstimation(const Eigen::MatrixXf& trajectory, float timestep)
    {
        Eigen::MatrixXf slope;
        slope.resize(trajectory.rows(), trajectory.cols());
        slope.fill(0);

        Eigen::VectorXf start = trajectory.col(0);
        Eigen::VectorXf initialEstimation(start.rows());
        initialEstimation.fill(0);

        DerivationEstimator<Eigen::VectorXf> estimator(start, initialEstimation);

        for (int i = 0; i < trajectory.cols() - 1; i++)
        {
            slope.col(i) = estimator.estimation;
            estimator.update(trajectory.col(i+1), timestep);
        }

        return slope;
    }

};

#endif

