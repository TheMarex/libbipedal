#ifndef _TRAJECTORY_EXPORTER_H_
#define _TRAJECTORY_EXPORTER_H_

#include <sstream>
#include <Eigen/Dense>
#include <VirtualRobot/Robot.h>
#include <MMM/Motion/Motion.h>

#include "Kinematics.h"

class TrajectoryExporter
{
public:
	TrajectoryExporter(VirtualRobot::RobotPtr robot,
		const std::string& pathToRobot,
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
		const std::vector<Kinematics::SupportPhase>& phase,
		float timestep)
	: robot(robot)
	, pathToRobot(pathToRobot)
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
    , phase(phase)
	{
	}

	void exportToMMM(const std::string& path);

private:

	VirtualRobot::RobotPtr robot;
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
    const std::vector<Kinematics::SupportPhase>& phase;
	float timestep;
};

#endif
