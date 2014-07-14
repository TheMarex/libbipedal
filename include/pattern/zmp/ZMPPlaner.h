#ifndef _ZMP_PLANER_H_
#define _ZMP_PLANER_H_

#include <VirtualRobot/VirtualRobot.h>
#include <boost/shared_ptr.hpp>

#include "../Utils/Kinematics.h"

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
    std::vector<Kinematics::SupportPhase> getSupportPhases() { return _mPhase; }

protected:
	FootstepPlanerPtr _pPlaner;
	bool _bComputed;
    Eigen::Matrix2Xf _mReference;                  // ZMPReference
    Eigen::Matrix3Xf _mCoM;                        // computed CoM
    Eigen::Matrix3Xf _mCoMVel;                     // computed CoM velocity
    Eigen::Matrix3Xf _mCoMAcc;                     // computed CoM acceleration
    Eigen::Matrix2Xf _mZMP;                        // computed "real" ZMP (resulting from CoM)
    std::vector<Kinematics::SupportPhase> _mPhase; // contains information about which leg should be in contact with the floor

	int _nSamplesDS;
	int _nSamplesSS;
};

typedef boost::shared_ptr<ZMPPlaner> ZMPPlanerPtr;

#endif

