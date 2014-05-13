#ifndef __POLYNOMIAL_FOOTSTEP_PLANER_H_
#define __POLYNOMIAL_FOOTSTEP_PLANER_H_

#include <boost/shared_ptr.hpp>

#include "FootstepPlaner.h"


class PolynomialFootstepPlaner :
	public FootstepPlaner
{
public:
	PolynomialFootstepPlaner();

	void setLeftFootFirst();
	void setRightFootFirst();

	virtual void setParameters(double stepLength, double stepPeriod, double doubleSupportPhase, double stepHeight, int sampleSize=100);

protected:
	virtual void computeFeetTrajectories(int numberOfSteps=5);
	int _iNumberOfSteps;
	bool _bParametersInitialized;
};

typedef boost::shared_ptr<PolynomialFootstepPlaner> PolynomialFootstepPlanerPtr;

#endif
