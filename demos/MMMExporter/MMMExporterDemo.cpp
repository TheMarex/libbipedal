#include <iostream>
#include <boost/filesystem.hpp>

#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/XML/RobotIO.h>

#include <Inventor/Qt/SoQt.h>

#include "FootstepPlaner.h"
#include "PolynomialFootstepPlaner.h"
#include "ZMP/ZMPPreviewControl.h"
#include "Utils/TrajectoryExporter.h"
#include "Utils/Kinematics.h"

using namespace VirtualRobot;

void run(const std::string& robotPath, const std::string& targetPath)
{
	VirtualRobot::RobotPtr robot = VirtualRobot::RobotIO::loadRobot(robotPath);

	PolynomialFootstepPlanerPtr planer(new PolynomialFootstepPlaner());
	planer->setRobotModel(robot);
	planer->generate(3);
	ZMPPreviewControl controller;
	controller.setFootstepPlaner(boost::dynamic_pointer_cast<FootstepPlaner>(planer));

	std::cout << "Comptuting reference..." << std::endl;
    controller.computeReference();

	std::cout << "Comptuting walking trajectory..." << std::endl;
	Eigen::MatrixXf trajectory;
	Kinematics::computeWalkingTrajectory(robot,
		controller.getCoMTrajectory(),
		planer->getRightFootTrajectory(),
		planer->getLeftFootTrajectory(),
		trajectory);

	const Eigen::Matrix3Xf comTrajectory = controller.getCoMTrajectory();
	const Eigen::Matrix2Xf zmpTrajectory = controller.getComputedZMPTrajectory();
	const Eigen::Matrix2Xf refZMPTrajectory = controller.getReferenceZMPTrajectory();

	TrajectoryExporter exporter(robot,
		robotPath,
		trajectory,
		planer->getLeftFootTrajectory(),
		comTrajectory,
		zmpTrajectory,
		refZMPTrajectory,
		1.0 / planer->getSamplesPerSecond());
	exporter.exportToMMM(targetPath);
}

int main(int argc, char *argv[])
{
	SoDB::init();
	SoQt::init(argc,argv,"stability demo");

	std::string filename("../../InvReachBipedal/data/armar4/ArmarIV-StartLeftLeg.xml");
	std::string motion("armar4_zmp_pattern.xml");

	if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
	{
		std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");
		if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
		{
			filename = robFile;
		}
	}

	boost::filesystem::path path(filename);
	if (!boost::filesystem::exists(path))
	{
		std::cout << "Error: Robot file not found: " << path.string() << std::endl;
		return 1;
	}
	filename = boost::filesystem::canonical(path).string();

	std::cout << "Using robot at " << filename << std::endl;

	run(filename, motion);

	return 0;

}
