/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __WALKING_IK_H__
#define __WALKING_IK_H__

#include <VirtualRobot/VirtualRobot.h>

#include "../bipedal.h"

namespace Bipedal
{

class WalkingIK
{
public:
    /**
     * \param outputNodeSet will be used to read out the result vector of angles
     * \param ikNodeSet nodes that will be used for the IK
     * \param colModelNodeSet nodes that will be used for computing the CoM
     */
    WalkingIK(const VirtualRobot::RobotPtr& robot,
              const VirtualRobot::RobotNodeSetPtr& outputNodeSet,
              const VirtualRobot::RobotNodeSetPtr& ikNodeSet,
              const VirtualRobot::RobotNodeSetPtr& colModelNodeSet,
              const VirtualRobot::RobotNodePtr& chest,
              const VirtualRobot::RobotNodePtr& pelvis,
              const VirtualRobot::RobotNodePtr& leftFootTCP,
              const VirtualRobot::RobotNodePtr& rightFootTCP)
    : robot(robot)
    , outputNodeSet(outputNodeSet)
    , ikNodeSet(ikNodeSet)
    , colModelNodeSet(colModelNodeSet)
    , chest(chest)
    , pelvis(pelvis)
    , leftFootTCP(leftFootTCP)
    , rightFootTCP(rightFootTCP)
    {
    }

    virtual void computeWalkingTrajectory(const Eigen::Matrix3Xf& comTrajectory,
                                          const Eigen::Matrix6Xf& rightFootTrajectory,
                                          const Eigen::Matrix6Xf& leftFootTrajectory,
                                          std::vector<Eigen::Matrix3f>& rootOrientation,
                                          Eigen::MatrixXf& trajectory) = 0;

protected:
    VirtualRobot::RobotPtr robot;
    VirtualRobot::RobotNodeSetPtr outputNodeSet;
    VirtualRobot::RobotNodeSetPtr ikNodeSet;
    VirtualRobot::RobotNodeSetPtr colModelNodeSet;
    VirtualRobot::RobotNodePtr chest;
    VirtualRobot::RobotNodePtr pelvis;
    VirtualRobot::RobotNodePtr leftFootTCP;
    VirtualRobot::RobotNodePtr rightFootTCP;
};

}

#endif
