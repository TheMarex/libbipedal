/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __TRAJECTORY_LOGGER_H__
#define __TRAJECTORY_LOGGER_H__

#include <VirtualRobot/VirtualRobot.h>

#include <MMM/Motion/Motion.h>
#include <MMM/Motion/MotionFrame.h>

namespace Bipedal
{

/**
 * Logs the motion that is done by the given robot on the given nodes
 * to an MMM motion for later replay.
 * The default framerate is 100 FPS, since the MMMViewer does not seem to
 * be able to do more. Note that this will cause subsampling effects (jerky motion).
 */
class TrajectoryLogger
{
public:
    TrajectoryLogger(const VirtualRobot::RobotPtr& robot,
                     const VirtualRobot::RobotNodeSetPtr& nodes,
                     const std::string& pathToRobot,
                     const std::string& path,
                     const std::string& motionName="Walking Simulation")
    : robot(robot)
    , pathToRobot(pathToRobot)
    , path(path)
    , motionName(motionName)
    , nodes(nodes)
    , framerate(100)
    , time(0)
    , lastFrameTime(0)
    {
    }

    ~TrajectoryLogger()
    {
        write(path);
    }

    void start();

    void write(const std::string& path);

    void recordFrame();

    void update(float dt);

private:
    double time;
    double lastFrameTime;
    unsigned framerate;
    VirtualRobot::RobotPtr robot;
    VirtualRobot::RobotNodeSetPtr nodes;
    MMM::MotionPtr motion;
    std::string motionName;
    std::string pathToRobot;
    std::string path;
};

}

#endif

