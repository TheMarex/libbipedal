#ifndef __POLYNOMIAL_FOOTSTEP_PLANER_H_
#define __POLYNOMIAL_FOOTSTEP_PLANER_H_

#include "footstepplaner.h"

class PolynomialFootstepPlaner :
	public FootstepPlaner
{
public:
	PolynomialFootstepPlaner(void);
	~PolynomialFootstepPlaner(void);

	void setParameters(double stepLength, double stepPeriod, double doubleSupportPhase, int sampleSize=100);
	void setLeftFootFirst();
	void setRightFootFirst();

	virtual void generate(int numberOfSteps=5);


protected:
	double _dStepLength;
	double _dStepWidth;
	double _dStepHeight;
	double _dStepPeriod;
	double _dSingleSupportPhase;
	double _dDoubleSupportPhase;
	double _iSampleSize;
	bool _bLeftFootFirst;
	int _iNumberOfSteps;
	
	bool _bParametersInitialized;

	double _dSS, _dDS;
	Eigen::Matrix3Xd _footTrajectory;
	Eigen::Matrix3Xd _footTrajectoryFirstLast;

};

#endif
