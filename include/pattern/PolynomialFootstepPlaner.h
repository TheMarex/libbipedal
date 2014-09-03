#ifndef __POLYNOMIAL_FOOTSTEP_PLANER_H_
#define __POLYNOMIAL_FOOTSTEP_PLANER_H_

#include <boost/shared_ptr.hpp>

#include "FootstepPlaner.h"


class PolynomialFootstepPlaner :
    public FootstepPlaner
{
public:
    PolynomialFootstepPlaner();

private:
    void computeStep(double ssTime,
                     double sampleDelta,
                     double stepLength,
                     double stepHeight,
                     Eigen::Matrix3Xf& trajectory);

    void computeGeneralizedTrajectories();

    Eigen::Matrix3Xf _mFootTrajectory;
    Eigen::Matrix3Xf _mFootTrajectoryFirst;
    Eigen::Matrix3Xf _mFootTrajectoryLast;

protected:
    virtual void computeFeetTrajectories() override;
};

typedef boost::shared_ptr<PolynomialFootstepPlaner> PolynomialFootstepPlanerPtr;

#endif
