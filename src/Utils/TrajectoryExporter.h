#ifndef _TRAJECTORY_EXPORTER_H_
#define _TRAJECTORY_EXPORTER_H_

#include <sstream>
#include <Eigen/Dense>
#include <VirtualRobot/Robot.h>
#include <MMM/Motion/Motion.h>

class TrajectoryExporter
{
public:
	TrajectoryExporter(VirtualRobot::RobotPtr robot,
		const std::string& pathToRobot,
		const Eigen::MatrixXf& bodyTrajectory,
		const Eigen::Matrix3Xf& leftFootTrajectory,
		const Eigen::Matrix3Xf& comTrajectory,
		const Eigen::Matrix3Xf& comVelocity,
		const Eigen::Matrix3Xf& comAcceleration,
		const Eigen::Matrix2Xf& computedZMPTrajectory,
		const Eigen::Matrix2Xf& referenceZMPTrajectory,
		float timestep)
	: robot(robot)
	, pathToRobot(pathToRobot)
	, bodyTrajectory(bodyTrajectory)
	, leftFootTrajectory(leftFootTrajectory)
	, comTrajectory(comTrajectory)
	, comVelocity(comVelocity)
	, comAcceleration(comAcceleration)
	, computedZMPTrajectory(computedZMPTrajectory)
	, referenceZMPTrajectory(referenceZMPTrajectory)
	, timestep(timestep)
	{
	}

	void exportToMMM(const std::string& path);

private:

	VirtualRobot::RobotPtr robot;
	const Eigen::MatrixXf& bodyTrajectory;
	const Eigen::Matrix3Xf& leftFootTrajectory;
	const Eigen::Matrix3Xf& comTrajectory;
	const Eigen::Matrix3Xf& comVelocity;
	const Eigen::Matrix3Xf& comAcceleration;
	const Eigen::Matrix2Xf& computedZMPTrajectory;
	const Eigen::Matrix2Xf& referenceZMPTrajectory;
	const std::string& pathToRobot;
	float timestep;
};

#endif
