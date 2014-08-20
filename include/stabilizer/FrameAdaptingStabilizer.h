#ifndef __FRAME_ADAPTING_STABILIZER_H_
#define __FRAME_ADAPTING_STABILIZER_H_

#include <Eigen/Dense>

class FrameAdaptingStabilizer
{
public:
    virtual const Eigen::Matrix4f& getChestPoseRef() = 0;
    virtual const Eigen::Matrix4f& getPelvisPoseRef() = 0;
    virtual const Eigen::Matrix4f& getLeftFootPoseRef() = 0;
    virtual const Eigen::Matrix4f& getRightFootPoseRef() = 0;
    virtual const Eigen::Vector3f& getCoMPositionRef() = 0;
    virtual const Eigen::Vector3f& getZMPPositionRef() = 0;

    virtual const Eigen::Matrix4f& getChestPose() = 0;
    virtual const Eigen::Matrix4f& getPelvisPose() = 0;
    virtual const Eigen::Matrix4f& getLeftFootPose() = 0;
    virtual const Eigen::Matrix4f& getRightFootPose() = 0;

    virtual const Eigen::VectorXf& getResultAngles() = 0;
    virtual const VirtualRobot::RobotNodeSetPtr& getNodes() = 0;

    virtual void update(float dt,
                        Kinematics::SupportPhase phase,
                        const Eigen::Vector3f& zmp,
                        const Eigen::Matrix4f& chestPoseRef,
                        const Eigen::Matrix4f& pelvisPoseRef,
                        const Eigen::Matrix4f& leftFootPoseRef,
                        const Eigen::Matrix4f& rightFootPoseRef,
                        const Eigen::Vector3f& comPositionRef) = 0;
};

#endif
