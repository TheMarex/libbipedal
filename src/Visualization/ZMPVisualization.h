#ifndef __ZMP_VISUALIZATION_H__
#define __ZMP_VISUALIZATION_H__

#include "../ZMP/ZMPPlaner.h"

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSwitch.h>

#include <Eigen/Dense>

#include <boost/shared_ptr.hpp>

class ZMPVisualization
{
public:
    ZMPVisualization(ZMPPlanerPtr planer);

	SoSeparator* getVisualization() {return _visualization;};
	void showReference(bool isVisible);
	void showRealZMP(bool isVisible);
	void showCoM(bool isVisible);

	void buildReferenceZMPVisualization();
	void buildComputedZMPVisualization();
	void buildCoMVisualization();

private:
	ZMPPlanerPtr planer;

	// root node for visualization
	SoSeparator* _visualization;
	SoSeparator* _referenceNodes;
	SoSeparator* _comNodes;
    SoSeparator* _realNodes;
    // switches to enable or disable visualization of referenceZMP/CoM-Trajectory/resulting-ZMP
	SoSwitch* _swReference;
	SoSwitch* _swCoM;
    SoSwitch* _swRealZMP;
};

typedef boost::shared_ptr<ZMPVisualization> ZMPVisualizationPtr;

#endif
