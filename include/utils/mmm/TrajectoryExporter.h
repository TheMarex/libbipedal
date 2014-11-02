/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef _TRAJECTORY_EXPORTER_H_
#define _TRAJECTORY_EXPORTER_H_

#include <sstream>
#include <Eigen/Dense>
#include <MMM/Motion/Motion.h>

#include "../Kinematics.h"
#include "../../bipedal.h"

namespace Bipedal
{

class TrajectoryExporter
{
public:
    TrajectoryExporter(const std::string& pathToRobot,
                       const std::vector<std::string> jointNames,
                       const Eigen::MatrixXf& bodyTrajectory,
                       const Eigen::Matrix3Xf& comTrajectory,
                       const Eigen::Matrix3Xf& comVelocity,
                       const Eigen::Matrix3Xf& comAcceleration,
                       const Eigen::Matrix2Xf& computedZMPTrajectory,
                       const Eigen::Matrix2Xf& referenceZMPTrajectory,
                       const std::vector<Eigen::Matrix4f>& leftFootTrajectory,
                       const std::vector<Eigen::Matrix4f>& rightFootTrajectory,
                       const std::vector<Eigen::Matrix4f>& chestTrajectory,
                       const std::vector<Eigen::Matrix4f>& pelvisTrajectory,
                       const std::vector<Bipedal::SupportInterval>& intervals,
                       float timestep)
        : pathToRobot(pathToRobot)
        , jointNames(jointNames)
        , bodyTrajectory(bodyTrajectory)
        , comTrajectory(comTrajectory)
        , comVelocity(comVelocity)
        , comAcceleration(comAcceleration)
        , computedZMPTrajectory(computedZMPTrajectory)
        , referenceZMPTrajectory(referenceZMPTrajectory)
        , leftFootTrajectory(leftFootTrajectory)
        , rightFootTrajectory(rightFootTrajectory)
        , chestTrajectory(chestTrajectory)
        , pelvisTrajectory(pelvisTrajectory)
        , timestep(timestep)
        , intervals(intervals)
    {
    }

    void exportToMMM(const std::string& path);

private:

    const std::vector<std::string> jointNames;
    const Eigen::MatrixXf& bodyTrajectory;
    const Eigen::Matrix3Xf& comTrajectory;
    const Eigen::Matrix3Xf& comVelocity;
    const Eigen::Matrix3Xf& comAcceleration;
    const Eigen::Matrix2Xf& computedZMPTrajectory;
    const Eigen::Matrix2Xf& referenceZMPTrajectory;
    const std::string& pathToRobot;
    const std::vector<Eigen::Matrix4f>& leftFootTrajectory;
    const std::vector<Eigen::Matrix4f>& rightFootTrajectory;
    const std::vector<Eigen::Matrix4f>& chestTrajectory;
    const std::vector<Eigen::Matrix4f>& pelvisTrajectory;
    const std::vector<Bipedal::SupportInterval>& intervals;
    float timestep;
};

}

#endif
