#ifndef __VELOCITY_ESTIMATION_H__
#define __VELOCITY_ESTIMATION_H__

#include <Eigen/Dense>

namespace VelocityEstimation
{
    inline Eigen::MatrixXf simpleDiff(const Eigen::MatrixXf& trajectory, float timestep)
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

    inline Eigen::MatrixXf neighboursDiff(const Eigen::MatrixXf& trajectory, float timestep)
    {
        Eigen::MatrixXf velocity;
        velocity.resize(trajectory.rows(), trajectory.cols());
        velocity.fill(0);

        for (int i = 1; i < trajectory.cols() - 1; i++)
        {
            velocity.col(i) = (trajectory.col(i + 1) - trajectory.col(i - 1)) / (2 * timestep);
        }

        return velocity;
    }
};

#endif

