/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __BALANCING_PLANER_H__
#define __BALANCING_PLANER_H__

#include <boost/shared_ptr.hpp>

#include "FootstepPlaner.h"

namespace Bipedal
{

class BalancingPlaner :
    public FootstepPlaner
{
public:
    BalancingPlaner(const VirtualRobot::RobotNodePtr& leftFootBody,
                    const VirtualRobot::RobotNodePtr& rightFootBody);

protected:
    void computeStep(double ssTime,
                     double sampleDelta,
                     double stepLength,
                     double stepHeight,
                     double lateralDirection,
                     Eigen::Matrix6Xf& trajectory);
    void computeLastStep(double sampleDelta,
                         double stepWidth,
                         double stepHeight,
                         double lateralDirection,
                         Eigen::Matrix6Xf& trajectory);
    void computeInitialStep(double sampleDelta,
                            double stepWidth,
                            double stepHeight,
                            double lateralDirection,
                            Eigen::Matrix6Xf& trajectory);

    void computeGeneralizedTrajectories();

    virtual void computeFeetTrajectories() override;

    Eigen::Matrix6Xf _mFootTrajectoryLeft;
    Eigen::Matrix6Xf _mFootTrajectoryRight;
    Eigen::Matrix6Xf _mFootTrajectoryFirst;
    Eigen::Matrix6Xf _mFootTrajectoryLast;
};

typedef boost::shared_ptr<BalancingPlaner> BalancingPlanerPtr;

}

#endif
