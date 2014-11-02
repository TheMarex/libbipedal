/*

Copyright (c) 2014, Ömer Terlemez, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __POLYNOMIAL_FOOTSTEP_PLANER_H_
#define __POLYNOMIAL_FOOTSTEP_PLANER_H_

#include <boost/shared_ptr.hpp>

#include "FootstepPlaner.h"

namespace Bipedal
{

class PolynomialFootstepPlaner :
    public FootstepPlaner
{
public:
    PolynomialFootstepPlaner(const VirtualRobot::RobotNodePtr& leftFootBody,
                            const VirtualRobot::RobotNodePtr& rightFootBody);

private:
    void computeStep(double ssTime,
                     double sampleDelta,
                     double stepLength,
                     double stepHeight,
                     Eigen::Matrix6Xf& trajectory);

    void computeGeneralizedTrajectories();

    Eigen::Matrix6Xf _mFootTrajectory;
    Eigen::Matrix6Xf _mFootTrajectoryFirst;
    Eigen::Matrix6Xf _mFootTrajectoryLast;

protected:
    virtual void computeFeetTrajectories() override;
};

typedef boost::shared_ptr<PolynomialFootstepPlaner> PolynomialFootstepPlanerPtr;

}

#endif
