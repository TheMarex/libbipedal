#ifndef _ZMP_PLANER_H_
#define _ZMP_PLANER_H_

#include <VirtualRobot/VirtualRobot.h>
#include <boost/shared_ptr.hpp>

#include "FootstepPlaner.h"

class ZMPPlaner
{
public:
	ZMPPlaner()
	: _bComputed(false)
	, _nSamplesDS(0)
	, _nSamplesSS(0)
	{
	}

	void setFootstepPlaner(FootstepPlanerPtr planer)
    {
		_pPlaner = planer;
    }

	virtual void computeReference() = 0;

    Eigen::Matrix3Xf getCoMTrajectory() { return _mCoM; }
    Eigen::Matrix3Xf getCoMVelocity() { return _mCoMVel; }
    Eigen::Matrix3Xf getCoMAcceleration() { return _mCoMAcc; }
    Eigen::Matrix2Xf getReferenceZMPTrajectory() { return _mReference; }
    Eigen::Matrix2Xf getComputedZMPTrajectory() { return _mZMP; }

protected:
	FootstepPlanerPtr _pPlaner;
	bool _bComputed;
    Eigen::Matrix2Xf _mReference;   // ZMPReference
    Eigen::Matrix3Xf _mCoM;         // computed CoM
    Eigen::Matrix3Xf _mCoMVel;      // computed CoM velocity
    Eigen::Matrix3Xf _mCoMAcc;      // computed CoM acceleration
    Eigen::Matrix2Xf _mZMP;         // computed "real" ZMP (resulting from CoM)

	int _nSamplesDS;
	int _nSamplesSS;
};

typedef boost::shared_ptr<ZMPPlaner> ZMPPlanerPtr;

#endif

