#include <fstream>
#include <string>
#include <sstream>

#include <Eigen/Dense>

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/Nodes/RobotNode.h>

#include <MMM/Motion/Motion.h>

#include <boost/filesystem.hpp>

#include "TrajectoryExporter.h"

/**
 * Limitation in boost::filesystem, Hack found on StackOverflow
 */
namespace boost {
namespace filesystem {
template < >
path& path::append< typename path::iterator >( typename path::iterator begin, typename path::iterator end, const codecvt_type& cvt)
{
	for( ; begin != end ; ++begin )
		*this /= *begin;
	return *this;
}

// Return path when appended to a_From will resolve to same as a_To
boost::filesystem::path make_relative( boost::filesystem::path a_From, boost::filesystem::path a_To )
{
	a_From = boost::filesystem::absolute( a_From ); a_To = boost::filesystem::absolute( a_To );
	boost::filesystem::path ret;
	boost::filesystem::path::const_iterator itrFrom( a_From.begin() ), itrTo( a_To.begin() );
	// Find common base
	for( boost::filesystem::path::const_iterator toEnd( a_To.end() ), fromEnd( a_From.end() ) ; itrFrom != fromEnd && itrTo != toEnd && *itrFrom == *itrTo; ++itrFrom, ++itrTo );
	// Navigate backwards in directory to reach previously found base
	for( boost::filesystem::path::const_iterator fromEnd( a_From.end() ); itrFrom != fromEnd; ++itrFrom )
	{
		if( (*itrFrom) != "." )
			ret /= "..";
	}
	// Now navigate down the directory branch
	ret.append( itrTo, a_To.end() );
	return ret;
}
}
}

/*
 * ----------------------------------------------------------------------------
 */

void TrajectoryExporter::exportToMMM(const std::string& path)
{
    VirtualRobot::RobotNodeSetPtr nodeSet = robot->getRobotNodeSet("Left2RightLeg");
	Eigen::Matrix4f rootPose = nodeSet->getKinematicRoot()->getGlobalPose();

	boost::filesystem::path targetPath(path);
	boost::filesystem::path baseDir = targetPath.parent_path();
	boost::filesystem::path robotPath(pathToRobot);
	boost::filesystem::path relRobotPath = make_relative(baseDir, robotPath);

	MMM::MotionPtr motion(new MMM::Motion("Walking pattern"));

	std::vector<VirtualRobot::RobotNodePtr> nodes = robot->getRobotNodes();
	std::vector<std::string> jointNames;
	for(auto& node : nodes)
		jointNames.push_back(node->getName());

	int size = bodyTrajectory.cols();
	int ndof = bodyTrajectory.rows();

	for (int i = 0; i < size; i++)
	{
		Eigen::Vector3f rootPos = leftFootTrajectory.col(i);
		Eigen::MatrixXf jointAngles = bodyTrajectory.col(i);
		MMM::MotionFramePtr frame(new MMM::MotionFrame(ndof));
		frame->setRootPose(rootPose);
		frame->setRootPos(rootPos);
		frame->joint = jointAngles;
		frame->timestep = timestep*i;
		motion->addMotionFrame(frame);
	}

	MMM::ModelPtr model(new MMM::Model());
	model->filename = relRobotPath.string();
	motion->setModel(model);

	std::ofstream out(path.c_str());
	out << "<MMM>"
	<< motion->toXML()
	<< "</MMM>";
}

