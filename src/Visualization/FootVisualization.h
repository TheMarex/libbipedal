#ifndef __FOOT_VISUALIZATION_H__
#define __FOOT_VISUALIZATION_H__

#include "../FootstepPlaner.h"

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSwitch.h>
#include <Inventor/nodes/SoMaterial.h>

#include <boost/shared_ptr.hpp>

class FootVisualization
{
public:
    FootVisualization(FootstepPlanerPtr planer);
    ~FootVisualization();

	void buildVisualization();
	SoSeparator* getVisualization();
	void showFootPositions(bool isVisible);
	void showFootTrajectories(bool isVisible);

private:
	void visualizeFeetShape();

	FootstepPlanerPtr planer;

	// visualization for the right foot (with border), without border, feet position and feet trajectory
	SoSeparator* _visuRightFoot;
	SoSeparator* _visuRightFootwoBorder;
	SoSeparator* _visuRightFootPositions;
	SoSeparator* _visuRightFootTrajectory;
	// visualization for the left foot (with border), without border, feet position and feet trajectory
	SoSeparator* _visuLeftFoot;
	SoSeparator* _visuLeftFootwoBorder;
	SoSeparator* _visuLeftFootPositions;
	SoSeparator* _visuLeftFootTrajectory;
	// root node for visualization
	SoSeparator* _visualization;
	// switches to enable or disable visualization of footstep positions and foot trajectories
	SoSwitch* _swPositions;
	SoSwitch* _swTrajectories;
};

typedef boost::shared_ptr<FootVisualization> FootVisualizationPtr;

#endif

