#ifndef __ZMP_REFERENCE_PLANNER_H__
#define __ZMP_REFERENCE_PLANNER_H__

#include <Eigen/Dense>
#include <vector>

#include "../../utils/Kinematics.h"

class ZMPReferencePlaner
{
public:
    /**
     * Given the position of each foot in each support phase
     * compute trajectory of the given support interval.
     */
    void generateReference(const Eigen::Matrix3Xf& leftFootPositions,
                           const Eigen::Matrix3Xf& rightFootPositions,
                           const std::vector<Kinematics::SupportInterval>& supportIntervals,
                           Eigen::Matrix2Xf& referenceZMP) const;
private:
    void shiftZMP(const Eigen::Vector2f& phaseBegin,
                  const Eigen::Vector2f& phaseEnd,
                  unsigned beginIdx,
                  unsigned endIdx,
                  Eigen::Matrix2Xf& referenceZMP) const;
    Eigen::Vector2f getRefPosition(Kinematics::SupportPhase phase,
                                   const Eigen::Vector2f& leftFoot,
                                   const Eigen::Vector2f& rigtFoot) const;
};

#endif
