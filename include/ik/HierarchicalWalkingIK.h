/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __HIERACHICAL_WALKING_IK_H__
#define __HIERACHICAL_WALKING_IK_H__

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/IK/HierarchicalIK.h>

#include "WalkingIK.h"

namespace Bipedal
{

/**
 * Computes the trajetories for the given joints to realize the given COM and foot
 * trajectories while maintaining an upright body posture.
 */
class HierarchicalWalkingIK : public WalkingIK
{
public:
    HierarchicalWalkingIK(const VirtualRobot::RobotPtr& robot,
                          const VirtualRobot::RobotNodeSetPtr& outputNodeSet,
                          const VirtualRobot::RobotNodeSetPtr& ikNodeSet,
                          const VirtualRobot::RobotNodeSetPtr& colModelNodeSet,
                          const VirtualRobot::RobotNodePtr& chest,
                          const VirtualRobot::RobotNodePtr& pelvis,
                          const VirtualRobot::RobotNodePtr& leftFootTCP,
                          const VirtualRobot::RobotNodePtr& rightFootTCP);

    virtual void computeWalkingTrajectory(const Eigen::Matrix3Xf& comTrajectory,
                                          const Eigen::Matrix6Xf& rightFootTrajectory,
                                          const Eigen::Matrix6Xf& leftFootTrajectory,
                                          std::vector<Eigen::Matrix3f>& rootOrientation,
                                          Eigen::MatrixXf& trajectory) override;

private:
    void computeStepConfiguration(const Eigen::Vector3f& targetCoM,
                                  const Eigen::Matrix4f& targetRightFootPose,
                                  const Eigen::Matrix4f& targetChestPose,
                                  const Eigen::Matrix4f& targetPelvisPose,
                                  Eigen::VectorXf& result);

    std::vector<VirtualRobot::HierarchicalIK::JacobiDefinition> jacobiDefinitions;
    VirtualRobot::HierarchicalIKPtr hIK;
    VirtualRobot::CoMIKPtr comIK;
    VirtualRobot::DifferentialIKPtr uprightBodyIK;
    VirtualRobot::DifferentialIKPtr straightPelvisIK;
    VirtualRobot::DifferentialIKPtr rightFootIK;
};

}

#endif
