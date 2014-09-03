#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/IK/HierarchicalIK.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/IK/CoMIK.h>
#include <Eigen/Dense>

#include "utils/Kinematics.h"

namespace Kinematics {

void extractControlFrames(VirtualRobot::RobotPtr robot,
                          const Eigen::Matrix3Xf& leftFootTrajectory,
                          const Eigen::MatrixXf&  bodyTrajectory,
                          VirtualRobot::RobotNodeSetPtr bodyJoints,
                          VirtualRobot::RobotNodePtr node,
                          std::vector<Eigen::Matrix4f>& controlFrames)
{
    Eigen::Matrix4f leftInitialPose = bodyJoints->getKinematicRoot()->getGlobalPose();
    int N = leftFootTrajectory.cols();

    for (int i = 0; i < N; i++)
    {
        // Move basis along with the left foot
        Eigen::Matrix4f leftFootPose = leftInitialPose;
        leftFootPose.block(0, 3, 3, 1) = 1000 * leftFootTrajectory.col(i);
        robot->setGlobalPose(leftFootPose);
        bodyJoints->setJointValues(bodyTrajectory.col(i));

        controlFrames.push_back(node->getGlobalPose());
    }
}

void transformTrajectoryToGroundFrame(VirtualRobot::RobotPtr robot,
                                      const Eigen::Matrix3Xf& leftFootTrajectory,
                                      VirtualRobot::RobotNodePtr leftFoot,
                                      VirtualRobot::RobotNodePtr rightFoot,
                                      VirtualRobot::RobotNodeSetPtr bodyJoints,
                                      const Eigen::MatrixXf& bodyTrajectory,
                                      const Eigen::Matrix3Xf& trajectory,
                                      const std::vector<Kinematics::SupportInterval>& intervals,
                                      Eigen::Matrix3Xf& relativeTrajectory)
{
    Eigen::Matrix4f leftInitialPose = bodyJoints->getKinematicRoot()->getGlobalPose();
    int N = trajectory.cols();
    relativeTrajectory.resize(3, N);

    auto intervalIter = intervals.begin();
    for (int i = 0; i < N; i++)
    {
        while (i >= intervalIter->endIdx)
        {
            intervalIter = std::next(intervalIter);
        }
        // Move basis along with the left foot
        Eigen::Matrix4f leftFootPose = leftInitialPose;
        leftFootPose.block(0, 3, 3, 1) = 1000 * leftFootTrajectory.col(i);
        robot->setGlobalPose(leftFootPose);
        bodyJoints->setJointValues(bodyTrajectory.col(i));
        Eigen::Matrix4f worldToRef = computeGroundFrame(leftFoot->getGlobalPose(), rightFoot->getGlobalPose(), intervalIter->phase);
        Eigen::Vector4f homVec;
        homVec(3, 0) = 1;
        homVec.block(0, 0, 3, 1) = trajectory.col(i) * 1000;
        relativeTrajectory.col(i) = worldToRef.colPivHouseholderQr().solve(homVec).block(0, 0, 3, 1) / 1000.0;
    }
}

}
