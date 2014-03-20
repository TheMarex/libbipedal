#ifndef _ZMP_PLANER_H_
#define _ZMP_PLANER_H_

#include <VirtualRobot/VirtualRobot.h>
#include <Inventor/nodes/SoSeparator.h>

#include "FootstepPlaner.h"

class ZMPPlaner 
{
public:
	ZMPPlaner();
	~ZMPPlaner();

public:
	void setFootstepPlaner(FootstepPlaner* planer) {_pPlaner = planer;};
	int getZMPPositions(Eigen::Matrix2Xf& zmp);

	virtual void computeReference() = 0;

	SoSeparator* getVisualization() {return _visualization;};
	void showReference(bool isVisible);
	void showRealZMP(bool isVisible);
	void showCoM(bool isVisible);

protected:
	FootstepPlaner* _pPlaner;
	bool _bComputed;
    Eigen::Matrix2Xf _mReference;   // ZMPReference
    Eigen::Matrix3Xf _mCoM;         // computed CoM
    Eigen::Matrix2Xf _mZMP;         // computed "real" ZMP (resulting from CoM)
	// root node for visualization
	SoSeparator* _visualization;
	SoSeparator* _referenceNodes;
	SoSeparator* _comNodes;
    SoSeparator* _realNodes;
    // switches to enable or disable visualization of referenceZMP/CoM-Trajectory/resulting-ZMP
	SoSwitch* _swReference;
	SoSwitch* _swCoM;
    SoSwitch* _swRealZMP;

	int _nSamplesDS;
	int _nSamplesSS;
};


#endif

