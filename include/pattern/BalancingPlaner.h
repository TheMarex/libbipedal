#ifndef __BALANCING_PLANER_H_
#define __BALANCING_PLANER_H_

#include <boost/shared_ptr.hpp>

#include "FootstepPlaner.h"

class BalancingPlaner :
    public FootstepPlaner
{
public:
    BalancingPlaner();

protected:
    void computeStep(double ssTime,
                     double sampleDelta,
                     double stepLength,
                     double stepHeight,
                     double lateralDirection,
                     Eigen::Matrix3Xf& trajectory);
    void computeLastStep(double sampleDelta,
                         double stepWidth,
                         double stepHeight,
                         double lateralDirection,
                         Eigen::Matrix3Xf& trajectory);
    void computeInitialStep(double sampleDelta,
                            double stepWidth,
                            double stepHeight,
                            double lateralDirection,
                            Eigen::Matrix3Xf& trajectory);

    void computeGeneralizedTrajectories();

    virtual void computeFeetTrajectories() override;

    Eigen::Matrix3Xf _mFootTrajectoryLeft;
    Eigen::Matrix3Xf _mFootTrajectoryRight;
    Eigen::Matrix3Xf _mFootTrajectoryFirst;
    Eigen::Matrix3Xf _mFootTrajectoryLast;
};

typedef boost::shared_ptr<BalancingPlaner> BalancingPlanerPtr;

#endif
