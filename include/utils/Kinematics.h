/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <Eigen/Dense>

namespace Eigen {
    typedef Matrix<float, 6, Dynamic> Matrix6Xf;
    typedef Matrix<float, 6, 1> Vector6f;
};

namespace Bipedal
{

enum SupportPhase : unsigned short
{
    SUPPORT_NONE  = 0,
    SUPPORT_LEFT  = 1,
    SUPPORT_RIGHT = 2,
    SUPPORT_BOTH  = 3
};

struct SupportInterval
{
    SupportInterval(unsigned beginIdx, unsigned endIdx, SupportPhase phase)
    : beginIdx(beginIdx), endIdx(endIdx), phase(phase) {}

    // start index of phase
    unsigned beginIdx;
    // end index of phase (exclusive)
    unsigned endIdx;
    SupportPhase phase;
};

/**
 * Detects the support phase based on the contact on the feet.
 */
class SupportPhaseSensor
{
public:
    SupportPhase phase;

    SupportPhaseSensor(const VirtualRobot::ContactSensorPtr& leftFootSensor,
                       const VirtualRobot::ContactSensorPtr& rightFootSensor);

    SupportPhase getContactPhase() const;

    void update(float dt);

private:
    VirtualRobot::ContactSensorPtr leftFootSensor;
    VirtualRobot::ContactSensorPtr rightFootSensor;
    double sameStateThreshold;
    double sameStateTime;
    SupportPhase nextPhase;
    bool initialized;
};

/**
 * Extract a vector of poses of the given node as the robot follows the given
 * angle trajectories.
 */
void extractControlFrames(VirtualRobot::RobotPtr robot,
                          const Eigen::Matrix6Xf& leftFootTrajectory,
                          const Eigen::MatrixXf&  bodyTrajectory,
                          VirtualRobot::RobotNodeSetPtr bodyJoints,
                          VirtualRobot::RobotNodePtr node,
                          std::vector<Eigen::Matrix4f>& controlFrames);

/**
 * Projects pose to ground (xy plane).
 */
inline Eigen::Matrix4f projectPoseToGround(const Eigen::Matrix4f& pose)
{
    Eigen::Matrix4f projected = Eigen::Matrix4f::Identity();
    projected.block(0, 0, 2, 2) = pose.block(0, 0, 2, 2);
    // norm x axsis
    projected.block(0, 0, 2, 1) /= projected.block(0, 0, 2, 1).norm();
    // norm y axsis
    projected.block(0, 1, 2, 1) /= projected.block(0, 1, 2, 1).norm();

    projected.block(0, 3, 2, 1) = pose.block(0, 3, 2, 1);

    return projected;
}

inline Eigen::Matrix3f poseFromXAxis(const Eigen::Vector3f& xAxis)
{
    Eigen::Matrix3f frame;
    Eigen::Vector3f zAxis(0, 0, 1);
    Eigen::Vector3f yAxis = zAxis.cross(xAxis);
    yAxis /= yAxis.norm();
    frame.block(0, 0, 3, 1) = xAxis;
    frame.block(0, 1, 3, 1) = yAxis;
    frame.block(0, 2, 3, 1) = zAxis;

    return frame;
}

inline Eigen::Matrix3f poseFromYAxis(const Eigen::Vector3f& yAxis)
{
    Eigen::Matrix3f frame;
    Eigen::Vector3f zAxis(0, 0, 1);
    Eigen::Vector3f xAxis = yAxis.cross(zAxis);
    xAxis /= xAxis.norm();
    frame.block(0, 0, 3, 1) = xAxis;
    frame.block(0, 1, 3, 1) = yAxis;
    frame.block(0, 2, 3, 1) = zAxis;

    return frame;
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
    Eigen::Matrix4f leftFootPoseProjected  = projectPoseToGround(leftFootPose);
    Eigen::Matrix4f rightFootPoseProjected = projectPoseToGround(rightFootPose);

    switch (phase)
    {
        case SUPPORT_LEFT:
            refToWorld = leftFootPoseProjected;
            break;

        case SUPPORT_RIGHT:
            refToWorld = rightFootPoseProjected;
            break;

        case SUPPORT_BOTH:
            refToWorld.block(0, 3, 3, 1) = (rightFootPoseProjected.block(0, 3, 3, 1) + leftFootPoseProjected.block(0, 3, 3, 1)) / 2.0;
            Eigen::Vector3f zAxis(0, 0, 1);
            Eigen::Vector3f yAxis = (rightFootPoseProjected.block(0, 1, 3, 1) + leftFootPoseProjected.block(0, 1, 3, 1)) / 2.0;
            yAxis /= yAxis.norm();
            refToWorld.block(0, 0, 3, 3) = poseFromYAxis(yAxis);
            break;
    }

    return refToWorld;
}

/**
 * Transforms trajectory to ground frame.
 *
 * Both trajectory and relativeTrajectory are in m.
 */
template<typename MatrixT>
inline void transformTrajectoryToGroundFrame(const VirtualRobot::RobotPtr& robot,
                                             const Eigen::Matrix6Xf& leftFootTrajectory,
                                             const VirtualRobot::RobotNodePtr& leftFoot,
                                             const VirtualRobot::RobotNodePtr& rightFoot,
                                             const VirtualRobot::RobotNodeSetPtr& bodyJoints,
                                             const Eigen::MatrixXf& bodyTrajectory,
                                             const MatrixT& trajectory,
                                             const std::vector<SupportInterval>& intervals,
                                             MatrixT& relativeTrajectory)
{
    Eigen::Matrix4f leftInitialPose = bodyJoints->getKinematicRoot()->getGlobalPose();
    int N = trajectory.cols();
    int M = trajectory.rows();
    relativeTrajectory.resize(M, N);

    BOOST_ASSERT(M > 0 && M <= 3);

    auto intervalIter = intervals.begin();
    for (int i = 0; i < N; i++)
    {
        while (i >= intervalIter->endIdx)
        {
            intervalIter = std::next(intervalIter);
        }

        // Move basis along with the left foot
        Eigen::Matrix4f leftFootPose = Eigen::Matrix4f::Identity();
        VirtualRobot::MathTools::posrpy2eigen4f(1000 * leftFootTrajectory.block(0, i, 3, 1),
                                                leftFootTrajectory.block(3, i, 3, 1),  leftFootPose);
        robot->setGlobalPose(leftFootPose);

        bodyJoints->setJointValues(bodyTrajectory.col(i));
        Eigen::Matrix4f worldToRef = computeGroundFrame(leftFoot->getGlobalPose(), rightFoot->getGlobalPose(), intervalIter->phase);
        Eigen::Vector4f homVec = Eigen::Vector4f::Zero();
        homVec(3, 0) = 1;
        homVec.block(0, 0, M, 1) = trajectory.col(i) * 1000;
        relativeTrajectory.block(0, i, M, 1) = worldToRef.colPivHouseholderQr().solve(homVec).block(0, 0, M, 1) / 1000.0;
    }
}

/**
 * Transforms orientation of vector to ground frame.
 *
 * Both trajectory and relativeTrajectory are in m.
 */
void transformOrientationToGroundFrame(const VirtualRobot::RobotPtr& robot,
                                       const Eigen::Matrix6Xf& leftFootTrajectory,
                                       const VirtualRobot::RobotNodePtr& leftFoot,
                                       const VirtualRobot::RobotNodePtr& rightFoot,
                                       const VirtualRobot::RobotNodeSetPtr& bodyJoints,
                                       const Eigen::MatrixXf& bodyTrajectory,
                                       const Eigen::Matrix3Xf& trajectory,
                                       const std::vector<SupportInterval>& intervals,
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
