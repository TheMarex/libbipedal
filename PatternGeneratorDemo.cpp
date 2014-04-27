#include <iostream>
#include <boost/filesystem.hpp>
#include <VirtualRobot/RuntimeEnvironment.h>

#include "PatternGeneratorWindow.h"

using namespace VirtualRobot;

int main(int argc, char *argv[])
{
	SoDB::init();
	SoQt::init(argc,argv,"stability demo");

	std::string filename("../../InvReachBipedal/data/armar4/ArmarIV-StartLeftLegColModel.xml");

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

	PatternGeneratorWindow rw(filename);
	rw.main();

	std::cout << " --- END --- " << std::endl;
	return 0;

}
