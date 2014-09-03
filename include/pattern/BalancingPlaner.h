#ifndef __BALANCING_PLANER_H_
#define __BALANCING_PLANER_H_

#include <boost/shared_ptr.hpp>

#include "PolynomialFootstepPlaner.h"

/**
 * FIXME The Footstep planer interface needs a cleanup.
 * This should not need to be a child of PolynomialFootstepPlaner
 * There is a very tight coupeling between the structure of the generated
 * trajectory and the ZMPPlaner.
 */
class BalancingPlaner :
    public PolynomialFootstepPlaner
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

    virtual void computeFeetTrajectories(int numberOfSteps = 5) override;
};

typedef boost::shared_ptr<BalancingPlaner> BalancingPlanerPtr;

#endif
