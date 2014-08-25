#ifndef __WALKING_IK_H__
#define __WALKING_IK_H__

#include <VirtualRobot/VirtualRobot.h>

class WalkingIK
{
public:
    WalkingIK(const VirtualRobot::RobotPtr& robot,
              const VirtualRobot::RobotNodeSetPtr& nodeSet,
              const VirtualRobot::RobotNodeSetPtr& colModelNodeSet,
              const VirtualRobot::RobotNodePtr& waist,
              const VirtualRobot::RobotNodePtr& leftFootTCP,
              const VirtualRobot::RobotNodePtr& rightFootTCP)
    : robot(robot)
    , nodeSet(nodeSet)
    , colModelNodeSet(colModelNodeSet)
    , waist(waist)
    , leftFootTCP(leftFootTCP)
    , rightFootTCP(rightFootTCP)
    {
    }

    virtual void computeWalkingTrajectory(const Eigen::Matrix3Xf& comTrajectory,
                                          const Eigen::Matrix3Xf& rightFootTrajectory,
                                          const Eigen::Matrix3Xf& leftFootTrajectory,
                                          Eigen::MatrixXf& trajectory) = 0;

protected:
    VirtualRobot::RobotPtr robot;
    VirtualRobot::RobotNodeSetPtr nodeSet;
    VirtualRobot::RobotNodeSetPtr colModelNodeSet;
    VirtualRobot::RobotNodePtr waist;
    VirtualRobot::RobotNodePtr leftFootTCP;
    VirtualRobot::RobotNodePtr rightFootTCP;
};

#endif
