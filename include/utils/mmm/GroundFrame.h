#ifndef __MMM_GROUND_FRAME_H__
#define __MMM_GROUND_FRAME_H__

#include <boost/optional.hpp>

#include "ControlPoint.h"
#include "ControlValue.h"
#include "ControlMatrix.h"
#include "../Kinematics.h"

namespace Bipedal
{
    inline boost::optional<Eigen::Vector3f> transformGroundFrameToGlobal(const MMM::MotionFramePtr& frame, const Eigen::Vector3f& inGroundFrame)
    {
        boost::optional<Eigen::Vector3f> ret;

        Eigen::Matrix4f leftFootPose;
        if (!GetControlMatrix(frame, "LeftFoot", leftFootPose))
            return ret;

        Eigen::Matrix4f rightFootPose;
        if (!GetControlMatrix(frame, "RightFoot", rightFootPose))
            return ret;

        Kinematics::SupportPhase phase;
        int val;
        if (!GetControlValue(frame, "SupportPhase", val))
            return ret;
        phase = static_cast<Kinematics::SupportPhase>(val);

        Eigen::Matrix4f groundFrame = Kinematics::computeGroundFrame(leftFootPose, rightFootPose, phase);
        Eigen::Vector4f homCom(inGroundFrame.x(), inGroundFrame.y(), inGroundFrame.z(), 1);
        Eigen::Vector3f inGlobalFrame = (groundFrame * homCom).block(0, 0, 3, 1);
        ret.reset(inGlobalFrame);

        return ret;
    }
};

#endif
