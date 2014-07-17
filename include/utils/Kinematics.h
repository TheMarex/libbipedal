#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <Eigen/Dense>

namespace Kinematics
{

enum SupportPhase : unsigned short
{
    SUPPORT_NONE  = 0,
    SUPPORT_LEFT  = 1,
    SUPPORT_RIGHT = 2,
    SUPPORT_BOTH  = 3
};

/**
 * Extract a vector of poses of the given node as the robot follows the given
 * angle trajectories.
 */
void extractControlFrames(VirtualRobot::RobotPtr robot,
                          const Eigen::Matrix3Xf& leftFootTrajectory,
                          const Eigen::MatrixXf&  bodyTrajectory,
                          VirtualRobot::RobotNodeSetPtr bodyJoints,
                          VirtualRobot::RobotNodePtr node,
                          std::vector<Eigen::Matrix4f>& controlFrames);

/**
 * Projects pose to ground (xy plane).
 */
inline Eigen::Matrix2f projectPoseToGround(const Eigen::Matrix4f& pose)
{
    Eigen::Matrix2f projected = pose.block(0, 0, 2, 2);
    // norm x axsis
    projected.block(0, 0, 2, 1) /= projected.block(0, 0, 2, 1).norm();
    // norm y axsis
    projected.block(0, 1, 2, 1) /= projected.block(0, 1, 2, 1).norm();

    return projected;
}

/**
 * Return pose of ground frame.
 *
 * Warning: Since the frame comes form simox it is in mm!
 */
inline Eigen::Matrix4f computeGroundFrame(const Eigen::Matrix4f& leftFootPose,
        const Eigen::Matrix4f& rightFootPose,
        SupportPhase phase)
{
    Eigen::Matrix4f refToWorld = Eigen::Matrix4f::Identity();

    switch (phase)
    {
        case SUPPORT_LEFT:
            refToWorld.block(0, 3, 2, 1) = leftFootPose.block(0, 3, 2, 1);
            refToWorld.block(0, 0, 2, 2) = projectPoseToGround(leftFootPose);
            break;

        case SUPPORT_RIGHT:
            refToWorld.block(0, 3, 2, 1) = rightFootPose.block(0, 3, 2, 1);
            refToWorld.block(0, 0, 2, 2) = projectPoseToGround(rightFootPose);
            break;

        case SUPPORT_BOTH:
            refToWorld.block(0, 3, 3, 1) = (rightFootPose.block(0, 3, 3, 1) + leftFootPose.block(0, 3, 3, 1)) / 2.0;
            Eigen::Vector3f zAxis(0, 0, 1);
            Eigen::Vector3f xAxis = (rightFootPose.block(0, 0, 3, 1) + leftFootPose.block(0, 0, 3, 1)) / 2.0;
            xAxis /= xAxis.norm();
            Eigen::Vector3f yAxis = zAxis.cross(xAxis);
            yAxis /= yAxis.norm();
            refToWorld.block(0, 0, 3, 1) = xAxis;
            refToWorld.block(0, 1, 3, 1) = yAxis;
            refToWorld.block(0, 2, 3, 1) = zAxis;
            break;
    }

    return refToWorld;
}

/**
 * Transforms trajectory to ground frame.
 *
 * Both trajectory and relativeTrajectory are in m.
 */
 void transformTrajectoryToGroundFrame(VirtualRobot::RobotPtr robot,
                                       const Eigen::Matrix3Xf& leftFootTrajectory,
                                       VirtualRobot::RobotNodePtr leftFoot,
                                       VirtualRobot::RobotNodePtr rightFoot,
                                       VirtualRobot::RobotNodeSetPtr bodyJoints,
                                       const Eigen::MatrixXf& bodyTrajectory,
                                       const Eigen::Matrix3Xf& trajectory,
                                       const std::vector<SupportPhase>& phase,
                                       Eigen::Matrix3Xf& relativeTrajectory);

/**
 * Set correct initial robot position *before* calling this function.
 * For example, extend the arms so they won't collide with the body.
 *
 * Expects the Left foot as root of the kinematic chain.
 * Right foot should be the TCP.
 * trajectory is the computed walking trajectory.
 */
void computeWalkingTrajectory(const VirtualRobot::RobotPtr& robot,
                              const VirtualRobot::RobotNodeSetPtr& nodeSet,
                              const VirtualRobot::RobotNodeSetPtr& colModelNodeSet,
                              const VirtualRobot::RobotNodePtr& waist,
                              const Eigen::Matrix3Xf& comTrajectory,
                              const Eigen::Matrix3Xf& rightFootTrajectory,
                              const Eigen::Matrix3Xf& leftFootTrajectory,
                              Eigen::MatrixXf& trajectory);

};

#endif
