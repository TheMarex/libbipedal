/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __MMM_GROUND_FRAME_H__
#define __MMM_GROUND_FRAME_H__

#include <boost/optional.hpp>

#include <VirtualRobot/MathTools.h>

#include "ControlPoint.h"
#include "ControlValue.h"
#include "ControlMatrix.h"
#include "../Kinematics.h"

namespace Bipedal
{
    /**
     * Unit of vector must the the same as the reference points LeftFoot and RightFoot.
     */
    inline boost::optional<Eigen::Vector3f> transformGroundFrameToGlobal(const MMM::MotionFramePtr& frame, const Eigen::Vector3f& inGroundFrame)
    {
        boost::optional<Eigen::Vector3f> ret;

        Eigen::Matrix4f leftFootPose;
        if (!GetControlMatrix(frame, "LeftFoot", leftFootPose))
            return ret;

        Eigen::Matrix4f rightFootPose;
        if (!GetControlMatrix(frame, "RightFoot", rightFootPose))
            return ret;

        Bipedal::SupportPhase phase;
        int val;
        if (!GetControlValue(frame, "SupportPhase", val))
            return ret;
        phase = static_cast<Bipedal::SupportPhase>(val);

        Eigen::Matrix4f groundFrame = Bipedal::computeGroundFrame(leftFootPose, rightFootPose, phase);
        Eigen::Vector3f inGlobalFrame = VirtualRobot::MathTools::transformPosition(inGroundFrame, groundFrame);
        ret.reset(inGlobalFrame);

        return ret;
    }
};

#endif
