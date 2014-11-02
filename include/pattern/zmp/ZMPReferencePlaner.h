/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __ZMP_REFERENCE_PLANNER_H__
#define __ZMP_REFERENCE_PLANNER_H__

#include <Eigen/Dense>
#include <vector>

#include "../../utils/Kinematics.h"
#include "../../bipedal.h"

namespace Bipedal
{

class ZMPReferencePlaner
{
public:
    /**
     * Given the position of each foot in each support phase
     * compute trajectory of the given support interval.
     */
    void generateReference(const Eigen::Matrix6Xf& leftFootTrajectory,
                           const Eigen::Matrix6Xf& rightFootTrajectory,
                           const std::vector<Bipedal::SupportInterval>& supportIntervals,
                           Eigen::Matrix2Xf& referenceZMP) const;
private:
    void shiftZMP(const Eigen::Vector2f& phaseBegin,
                  const Eigen::Vector2f& phaseEnd,
                  unsigned beginIdx,
                  unsigned endIdx,
                  Eigen::Matrix2Xf& referenceZMP) const;
    Eigen::Vector2f getRefPosition(Bipedal::SupportPhase phase,
                                   const Eigen::Vector2f& leftFoot,
                                   const Eigen::Vector2f& rigtFoot) const;
};

}

#endif
