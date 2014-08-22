#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>

#include <MMM/Motion/Motion.h>
#include <MMM/Motion/MotionFrame.h>

#include <boost/assert.hpp>
#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>

#include <fstream>

#include "utils/mmm/TrajectoryLogger.h"


void TrajectoryLogger::start()
{
    motion = boost::make_shared<MMM::Motion>(motionName);

    boost::filesystem::path targetPath(path);
    boost::filesystem::path baseDir = targetPath.parent_path();
    std::string relRobotPath = MMM::XML::make_relative(baseDir.string(), pathToRobot);

    MMM::ModelPtr model(new MMM::Model());
    model->filename = relRobotPath;
    motion->setModel(model);

    std::vector<std::string> jointNames;
    for (auto& n: *nodes)
    {
        jointNames.push_back(n->getName());
    }
    motion->setJointOrder(jointNames);
}

void TrajectoryLogger::write(const std::string& path)
{
    std::ofstream out(path.c_str());
    out << "<MMM>"
        << motion->toXML()
        << "</MMM>";
}

void TrajectoryLogger::recordFrame()
{
    MMM::MotionFramePtr frame(new MMM::MotionFrame(nodes->getSize()));
    Eigen::Matrix4f rootPose = robot->getRootNode()->getGlobalPose();
    frame->setRootPose(rootPose);
    frame->setRootPos(rootPose.block(0, 3, 3, 1));
    nodes->getJointValues(frame->joint);
    frame->timestep = time;
    motion->addMotionFrame(frame);
}

void TrajectoryLogger::update(float dt)
{
    if (!motion)
        return;

    if (time - lastFrameTime > 1.0 / framerate)
    {
        lastFrameTime = time;
        recordFrame();
    }

    time += dt;
}

