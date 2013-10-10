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

protected:
	FootstepPlaner* _pPlaner;
	bool _bComputed;
	Eigen::Matrix2Xf _mReference;
	Eigen::Matrix2Xf _mZMP;
	// root node for visualization
	SoSeparator* _visualization;
	SoSeparator* _referenceNodes;
	SoSeparator* _realNodes;
	// switches to enable or disable visualization of footstep positions and foot trajectories
	SoSwitch* _swReference;
	SoSwitch* _swRealZMP;
};


#endif

