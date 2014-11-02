/*

Copyright (c) 2014, Ömer Terlemez, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef _ZMP_PLANER_H_
#define _ZMP_PLANER_H_

#include <VirtualRobot/VirtualRobot.h>
#include <boost/shared_ptr.hpp>

#include "../../bipedal.h"
#include "../../utils/Kinematics.h"
#include "../FootstepPlaner.h"
#include "ZMPReferencePlaner.h"

namespace Bipedal
{

class ZMPPlaner
{
public:

    ZMPPlaner(const FootstepPlanerPtr& footstepPlaner,
              const ZMPReferencePlanerPtr& refPlaner,
              double comHeight=0.86)
        : _bComputed(false)
        , _pPlaner(footstepPlaner)
        , _pRefPlaner(refPlaner)
        , _dCoMHeight(comHeight)
    {
    }

    const Eigen::Matrix3Xf& getCoMTrajectory()
    {
        BOOST_ASSERT(_bComputed);
        return _mCoM;
    }
    const Eigen::Matrix3Xf& getCoMVelocity()
    {
        BOOST_ASSERT(_bComputed);
        return _mCoMVel;
    }
    const Eigen::Matrix3Xf& getCoMAcceleration()
    {
        BOOST_ASSERT(_bComputed);
        return _mCoMAcc;
    }
    const Eigen::Matrix2Xf& getReferenceZMPTrajectory()
    {
        BOOST_ASSERT(_bComputed);
        return _mReference;
    }
    const Eigen::Matrix2Xf& getComputedZMPTrajectory()
    {
        BOOST_ASSERT(_bComputed);
        return _mZMP;
    }

    void generate(const Eigen::Matrix4f& leftFootPose, const Eigen::Matrix4f& rightFootPose)
    {
        _pPlaner->generate(leftFootPose, rightFootPose);
        _pRefPlaner->generateReference(_pPlaner->getLeftFootTrajectory(),
                                       _pPlaner->getRightFootTrajectory(),
                                       _pPlaner->getSupportIntervals(),
                                       _mReference);
        computeCoMTrajectory();

        _bComputed = true;
    }

protected:
    virtual void computeCoMTrajectory() = 0;

    FootstepPlanerPtr _pPlaner;
    ZMPReferencePlanerPtr _pRefPlaner;

    double _dCoMHeight;
    bool _bComputed;

    Eigen::Matrix3Xf _mCoM;                        // computed CoM
    Eigen::Matrix3Xf _mCoMVel;                     // computed CoM velocity
    Eigen::Matrix3Xf _mCoMAcc;                     // computed CoM acceleration
    Eigen::Matrix2Xf _mZMP;                        // computed "real" ZMP (resulting from CoM)
    Eigen::Matrix2Xf _mReference;                  // reference ZMP
    std::vector<Bipedal::SupportPhase> _mPhase; // contains information about which leg should be in contact with the floor
};

}

#endif

