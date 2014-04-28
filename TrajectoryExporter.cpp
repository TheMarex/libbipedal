#include <fstream>
#include <string>
#include <sstream>

#include <Eigen/Dense>

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/Nodes/RobotNode.h>

#include <MMM/Motion/Motion.h>
#include <MMM/XMLTools.h>

#include <boost/filesystem.hpp>

#include "TrajectoryExporter.h"

void TrajectoryExporter::exportToMMM(const std::string& path)
{
    VirtualRobot::RobotNodeSetPtr nodeSet = robot->getRobotNodeSet("Left2RightLeg");
	Eigen::Matrix4f rootPose = nodeSet->getKinematicRoot()->getGlobalPose();

	boost::filesystem::path targetPath(path);
	boost::filesystem::path baseDir = targetPath.parent_path();
	std::string relRobotPath = MMM::XML::make_relative(baseDir.string(), pathToRobot);

	MMM::MotionPtr motion(new MMM::Motion("Walking pattern"));

	std::vector<VirtualRobot::RobotNodePtr> nodes = robot->getRobotNodes();
	std::vector<std::string> jointNames;
	for(auto& node : nodes)
		jointNames.push_back(node->getName());

	int size = bodyTrajectory.cols();
	int ndof = bodyTrajectory.rows();

	for (int i = 0; i < size; i++)
	{
		// we need rootPos in mm
		Eigen::Vector3f rootPos = 1000 * leftFootTrajectory.col(i);
		Eigen::MatrixXf jointAngles = bodyTrajectory.col(i);
		MMM::MotionFramePtr frame(new MMM::MotionFrame(ndof));
		frame->setRootPose(rootPose);
		frame->setRootPos(rootPos);
		frame->joint = jointAngles;
		frame->timestep = timestep*i;
		motion->addMotionFrame(frame);
	}

	MMM::ModelPtr model(new MMM::Model());
	model->filename = relRobotPath;
	motion->setModel(model);

	std::ofstream out(path.c_str());
	out << "<MMM>"
	<< motion->toXML()
	<< "</MMM>";
}

