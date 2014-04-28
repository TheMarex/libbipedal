#ifndef _TRAJECTORY_EXPORTER_H_
#define _TRAJECTORY_EXPORTER_H_

#include <sstream>
#include <VirtualRobot/Robot.h>
#include <Eigen/Dense>

class TrajectoryExporter
{
public:
	TrajectoryExporter(VirtualRobot::RobotPtr robot,
		const std::string& pathToRobot,
		const Eigen::MatrixXf& bodyTrajectory,
		const Eigen::Matrix3Xf& leftFootTrajectory,
		float timestep)
	: robot(robot)
	, pathToRobot(pathToRobot)
	, bodyTrajectory(bodyTrajectory)
	, leftFootTrajectory(leftFootTrajectory)
	, timestep(timestep)
	{
	}

	void exportToMMM(const std::string& path);

private:
	VirtualRobot::RobotPtr robot;
	const Eigen::MatrixXf& bodyTrajectory;
	const Eigen::Matrix3Xf& leftFootTrajectory;
	const std::string& pathToRobot;
	float timestep;
};

#endif
