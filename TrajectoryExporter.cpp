#include <fstream>
#include <string>
#include <sstream>

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <Eigen/Dense>
#include <MMM/Motion/Motion.h>

#include "TrajectoryExporter.h"

void TrajectoryExporter::exportToMMM(const std::string& path)
{
    VirtualRobot::RobotNodeSetPtr nodeSet = robot->getRobotNodeSet("Left2RightLeg");
	Eigen::Matrix4f rootPose = nodeSet->getKinematicRoot()->getGlobalPose();

	MMM::MotionPtr motion(new MMM::Motion("Walking pattern"));

	std::vector<VirtualRobot::RobotNodePtr> nodes = robot->getRobotNodes();
	std::vector<std::string> jointNames;
	for(auto& node : nodes)
		jointNames.push_back(node->getName());

	// FIXME
	float timestep = 1.0f;

	int size = bodyTrajectory.cols();
	int ndof = bodyTrajectory.rows();

	for (int i = 0; i < size; i++)
	{
		Eigen::Vector3f rootPos = leftFootTrajectory.col(i);
		Eigen::MatrixXf jointAngles = bodyTrajectory.col(i);
		MMM::MotionFramePtr frame(new MMM::MotionFrame(ndof));
		frame->setRootPos(rootPos);
		frame->setRootPose(rootPose);
		frame->joint = jointAngles;
		frame->timestep = timestep;
		motion->addMotionFrame(frame);
	}

	std::ofstream out(path.c_str());
	out << "<MMM>" << motion->toXML() << "</MMM>";
}

