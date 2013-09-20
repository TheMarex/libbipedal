#ifndef __Footstep_Planer_H_
#define __Footstep_Planer_H_

#include <VirtualRobot/VirtualRobot.h>

class FootstepPlaner
{
public:
	FootstepPlaner();
	~FootstepPlaner();

	virtual void generate(int numberOfSteps=5) = 0;

protected:
	Eigen::Matrix3Xd _mLFootTrajectory;
	Eigen::Matrix3Xd _mRFootTrajectory;
};

#endif
