#ifndef __HIERACHICAL_WALKING_IK_H__
#define __HIERACHICAL_WALKING_IK_H__

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/IK/HierarchicalIK.h>

#include "WalkingIK.h"

/**
 * Computes the trajetories for the given joints to realize the given COM and foot
 * trajectories while maintaining an upright body posture.
 */
class HierarchicalWalkingIK : public WalkingIK
{
public:
    HierarchicalWalkingIK(const VirtualRobot::RobotPtr& robot,
                          const VirtualRobot::RobotNodeSetPtr& nodeSet,
                          const VirtualRobot::RobotNodeSetPtr& colModelNodeSet,
                          const VirtualRobot::RobotNodePtr& waist,
                          const VirtualRobot::RobotNodePtr& leftFootTCP,
                          const VirtualRobot::RobotNodePtr& rightFootTCP);

    virtual void computeWalkingTrajectory(const Eigen::Matrix3Xf& comTrajectory,
                                          const Eigen::Matrix3Xf& rightFootTrajectory,
                                          const Eigen::Matrix3Xf& leftFootTrajectory,
                                          std::vector<Eigen::Matrix3f>& rootOrientation,
                                          Eigen::MatrixXf& trajectory) override;

private:
    void computeStepConfiguration(const Eigen::Vector3f& targetCoM,
                                  const Eigen::Matrix4f& targetRightFootPose,
                                  const Eigen::Matrix4f& targetWaistPose,
                                  Eigen::VectorXf& result);

    std::vector<VirtualRobot::HierarchicalIK::JacobiDefinition> jacobiDefinitions;
    VirtualRobot::HierarchicalIKPtr hIK;
    VirtualRobot::CoMIKPtr comIK;
    VirtualRobot::DifferentialIKPtr uprightBodyIK;
    VirtualRobot::DifferentialIKPtr rightFootIK;
};

#endif
