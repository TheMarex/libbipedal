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
    void calculateStep(double ssTime,
                       int ssSamples,
                       double sampleDelta,
                       double stepLength,
                       double stepHeight,
                       double lateralDirection,
                       Eigen::Matrix3Xf& trajectory);
    void calculateLastStep(double sampleDelta,
                           double stepWidth,
                           double stepHeight,
                           double lateralDirection,
                           Eigen::Matrix3Xf& trajectory);
    void calculateInitialStep(double sampleDelta,
                              double stepWidth,
                              double stepHeight,
                              double lateralDirection,
                              Eigen::Matrix3Xf& trajectory);

    virtual void computeFeetTrajectories() override;
};

typedef boost::shared_ptr<BalancingPlaner> BalancingPlanerPtr;

#endif
