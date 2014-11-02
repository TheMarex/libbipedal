/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __REFERENCE_IK_H__
#define __REFERENCE_IK_H__

#include <Eigen/Dense>
#include <VirtualRobot/VirtualRobot.h>

namespace Bipedal
{


/* Interface to compute IK from reference frames */
class ReferenceIK
{
public:
    virtual bool computeStep(const Eigen::Matrix4f& leftFootPose,
                             const Eigen::Matrix4f& rightFootPose,
                             const Eigen::Matrix4f& chestPose,
                             const Eigen::Matrix4f& pelvisPose,
                             const Eigen::Vector3f& comPosition,
                             Bipedal::SupportPhase phase,
                             Eigen::VectorXf &result) = 0;

    virtual const VirtualRobot::RobotNodeSetPtr& getNodes() = 0;
};

}

#endif

