#include "pattern/zmp/ZMPReferencePlaner.h"

/**
* Starting with DS-Phase we alternate between DS and SS Phase
* for every phase we create a reference-ZMP... in SS Phase it is in the center of the standing leg,
* in DS-Phase it has to move from previous standing leg to previous swing leg
* exceptions are the first and last steps...
* here we have to move the zmp to the standing foot or center of both feet accordingly
*/
void ZMPReferencePlaner::generateReference(const Eigen::Matrix3Xf& leftFootPositions,
                                         const Eigen::Matrix3Xf& rightFootPositions,
                                         const std::vector<Kinematics::SupportInterval>& supportIntervals,
                                         Eigen::Matrix2Xf& referenceZMP) const
{
    unsigned columns = supportIntervals.back().endIdx;
    referenceZMP.resize(2, columns);

    std::cout << "Num intervals: " << supportIntervals.size() << std::endl;

    const unsigned size = supportIntervals.size();
    for (unsigned i = 0; i < size; i++)
    {
        const auto& interval     = supportIntervals[i];
        const auto& prevInterval = i > 0 ? supportIntervals[i-1] : interval;
        const auto& nextInterval = i < size - 1 ? supportIntervals[i+1] : interval;
        const auto& currentLeft  = leftFootPositions.block(0, i, 2, 1);
        const auto& currentRight = rightFootPositions.block(0, i, 2, 1);

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

    std::cout << "Shifting from [" << begin.transpose() << "] to [" << end.transpose() << "] : (" << beginIdx << ", " << midIdx << ", " << endIdx << ")" << std::endl;

    for (unsigned i = beginIdx; i < midIdx; i++)
    {
        referenceZMP.block(0, i, 2, 1) = begin;
    }
    for (unsigned i = midIdx; i < endIdx; i++)
    {
        referenceZMP.block(0, i, 2, 1) = end;
    }
}


