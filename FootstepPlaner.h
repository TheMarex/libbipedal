#ifndef __Footstep_Planer_H_
#define __Footstep_Planer_H_

#include <VirtualRobot/VirtualRobot.h>

class FootstepPlaner
{
public:
	FootstepPlaner();
	~FootstepPlaner();

	virtual void generate(int numberOfSteps=5) = 0;
	bool isCalculated() {return _bCalculated;};
	
	Eigen::Matrix3Xd const getLeftFootPositions() {return _mLFootPositions;};
	Eigen::Matrix3Xd const getRightFootPositions() {return _mRFootPositions;};

protected:
	Eigen::Matrix3Xd _mLFootTrajectory;
	Eigen::Matrix3Xd _mRFootTrajectory;

	Eigen::Matrix3Xd _mLFootPositions;
	Eigen::Matrix3Xd _mRFootPositions;

	bool _bCalculated;
};

#endif
