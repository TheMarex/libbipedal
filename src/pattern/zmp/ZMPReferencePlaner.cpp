#include "pattern/zmp/ZMPReferencePlaner.h"

void ZMPReferencePlaner::generateReference(const Eigen::Matrix6Xf& leftFootTrajectory,
                                           const Eigen::Matrix6Xf& rightFootTrajectory,
                                           const std::vector<Kinematics::SupportInterval>& supportIntervals,
                                           Eigen::Matrix2Xf& referenceZMP) const
{
    unsigned columns = supportIntervals.back().endIdx;
    referenceZMP.resize(2, columns);

    const unsigned size = supportIntervals.size();
    for (unsigned i = 0; i < size; i++)
    {
        const auto& interval     = supportIntervals[i];
        const auto& prevInterval = i > 0 ? supportIntervals[i-1] : interval;
        const auto& nextInterval = i < size - 1 ? supportIntervals[i+1] : interval;
        // foot poses at the begin of the interval, in SS one stays constant, in DS both
        const auto& currentLeft  = leftFootTrajectory.block(0, interval.beginIdx, 2, 1);
        const auto& currentRight = rightFootTrajectory.block(0, interval.beginIdx, 2, 1);

        switch (interval.phase)
        {
            case Kinematics::SUPPORT_BOTH:
                // shift ZMP from last support foot to next support foot
                shiftZMP(getRefPosition(prevInterval.phase, currentLeft, currentRight),
                         getRefPosition(nextInterval.phase, currentLeft, currentRight),
                         interval.beginIdx,
                         interval.endIdx,
                         referenceZMP);
            break;
            case Kinematics::SUPPORT_LEFT:
                // ZMP needs to remain at left foot
                for (unsigned j = interval.beginIdx; j < interval.endIdx; j++)
                {
                    referenceZMP.block(0, j, 2, 1) = currentLeft;
                }
            break;
            case Kinematics::SUPPORT_RIGHT:
                // ZMP needs to remain at right foot
                for (unsigned j = interval.beginIdx; j < interval.endIdx; j++)
                {
                    referenceZMP.block(0, j, 2, 1) = currentRight;
                }
            break;
        }
    }
}

Eigen::Vector2f ZMPReferencePlaner::getRefPosition(Kinematics::SupportPhase phase,
                                                   const Eigen::Vector2f& leftFoot,
                                                   const Eigen::Vector2f& rightFoot) const
{
    Eigen::Vector2f zmp;
    switch (phase)
    {
        case Kinematics::SUPPORT_LEFT:
            zmp = leftFoot;
        break;
        case Kinematics::SUPPORT_RIGHT:
            zmp = rightFoot;
        break;
        case Kinematics::SUPPORT_BOTH:
            zmp = (leftFoot + rightFoot) / 2;
        break;
    }

    return zmp;
}


void ZMPReferencePlaner::shiftZMP(const Eigen::Vector2f& begin,
                                  const Eigen::Vector2f& end,
                                  unsigned beginIdx,
                                  unsigned endIdx,
                                  Eigen::Matrix2Xf& referenceZMP) const
{
    unsigned numCols = endIdx - beginIdx;
    unsigned midIdx = beginIdx + numCols / 2;

    for (unsigned i = beginIdx; i < midIdx; i++)
    {
        referenceZMP.block(0, i, 2, 1) = begin;
    }
    for (unsigned i = midIdx; i < endIdx; i++)
    {
        referenceZMP.block(0, i, 2, 1) = end;
    }
}


