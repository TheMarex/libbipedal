#ifndef _TRAJECTORY_EXPORTER_H_
#define _TRAJECTORY_EXPORTER_H_

#include <sstream>
#include <VirtualRobot/Robot.h>
#include <Eigen/Dense>

class TrajectoryExporter
{
public:
	TrajectoryExporter(VirtualRobot::RobotPtr robot,
		const Eigen::MatrixXf& bodyTrajectory,
		const Eigen::Matrix3Xf& leftFootTrajectory)
	: robot(robot)
	, bodyTrajectory(bodyTrajectory)
	, leftFootTrajectory(leftFootTrajectory)
	{
	}

	void exportToMMM(const std::string& path);

private:
	VirtualRobot::RobotPtr robot;
	const Eigen::MatrixXf& bodyTrajectory;
	const Eigen::Matrix3Xf& leftFootTrajectory;
};

#endif
