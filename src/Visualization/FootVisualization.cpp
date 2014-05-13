#include "FootVisualization.h"

#include "VisualizationUtils.h"

#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <Eigen/Dense>

FootVisualization::FootVisualization(FootstepPlanerPtr planer)
: planer(planer)
{
	_visualization = new SoSeparator();
	_visuRightFoot = new SoSeparator();
	_visuRightFootwoBorder = new SoSeparator();
	_visuRightFootPositions  = new SoSeparator();
	_visuRightFootTrajectory = new SoSeparator();
	_visuLeftFoot = new SoSeparator();
	_visuLeftFootwoBorder = new SoSeparator();
	_visuLeftFootPositions = new SoSeparator();
	_visuLeftFootTrajectory = new SoSeparator();

	_swPositions = new SoSwitch();
	_swTrajectories = new SoSwitch();
	_swPositions->whichChild=SO_SWITCH_NONE;
	_swTrajectories->whichChild=SO_SWITCH_NONE;

	_visualization->ref();

	// build graph
	_visualization->addChild(_swPositions);
	_visualization->addChild(_swTrajectories);
	_swPositions->addChild(_visuRightFootPositions);
	_swPositions->addChild(_visuLeftFootPositions);

	_swTrajectories->addChild(_visuRightFootTrajectory);
	_swTrajectories->addChild(_visuLeftFootTrajectory);
}

FootVisualization::~FootVisualization()
{
	_visualization->removeAllChildren();
	_visuRightFoot->unref();
	_visuLeftFoot->unref();
	_visualization->unref();
}

// compute the shape of each foot and generate a visualization for every foot
void FootVisualization::visualizeFeetShape()
{
	// delete old visualizations
	_visuRightFoot->removeAllChildren();
	_visuLeftFoot->removeAllChildren();
	// build visualisation template for each foot (levitated about 2.5f in z-direction for better visibility)
	VirtualRobot::MathTools::Plane plane =  VirtualRobot::MathTools::getFloorPlane();
	_visuRightFoot = VirtualRobot::CoinVisualizationFactory::CreateConvexHull2DVisualization(planer->getRightFootConvexHull(), plane, 
		VirtualRobot::VisualizationFactory::Color::Blue(), VirtualRobot::VisualizationFactory::Color::Red(),6.0f,Eigen::Vector3f(0,0,2.5f));
	_visuLeftFoot = VirtualRobot::CoinVisualizationFactory::CreateConvexHull2DVisualization(planer->getLeftFootConvexHull(), plane, 
		VirtualRobot::VisualizationFactory::Color::Green(), VirtualRobot::VisualizationFactory::Color::Red(),6.0f,Eigen::Vector3f(0,0,2.5f));
	// ref(), otherwise the Shapes get deleted!
	_visuRightFoot->ref();
	_visuLeftFoot->ref();
	// build a different representation of the feet shape for trajectory visualization
	_visuRightFootwoBorder = VirtualRobot::CoinVisualizationFactory::CreateConvexHull2DVisualization(planer->getRightFootConvexHull(), plane, 
		VirtualRobot::VisualizationFactory::Color::Blue(0.9f), VirtualRobot::VisualizationFactory::Color::Gray(),0.1f,Eigen::Vector3f(0,0,2.5f));
	_visuLeftFootwoBorder = VirtualRobot::CoinVisualizationFactory::CreateConvexHull2DVisualization(planer->getLeftFootConvexHull(), plane, 
		VirtualRobot::VisualizationFactory::Color::Green(0.9f), VirtualRobot::VisualizationFactory::Color::Gray(),0.1f,Eigen::Vector3f(0,0,2.5f));
	_visuRightFootwoBorder->ref();
	_visuLeftFootwoBorder->ref();
	// build the visualization
	std::cout << "Shape of Feet computed!" << std::endl;
}

// build visualization for feet position and feet trajectories
void FootVisualization::buildVisualization() {
	visualizeFeetShape();

	// clean up old visualizations
	std::cout << "Number of Children of _visuRightFootPositions before: " << _visuRightFootPositions->getNumChildren();
	_visuRightFootPositions->removeAllChildren();
	_visuLeftFootPositions->removeAllChildren();
	_visuRightFootTrajectory->removeAllChildren();
	_visuLeftFootTrajectory->removeAllChildren();
	std::cout << ", and after: " << _visuRightFootPositions->getNumChildren() << std::endl;

	//_visualization->addChild(_visuLeftFoot);
	/*SoSeparator* spSep = new SoSeparator();
	SoSphere* sp = new SoSphere();
	sp->radius = 0.02;
	spSep->addChild(sp);
	generateVisualizationDuplicatesFromTrajectories(_visuRightFootPositions, spSep, _mRFootPositions);
	*/

	// visualize foot positions
	VisualizationUtils::generateVisualizationDuplicatesFromTrajectories(_visuRightFootPositions, _visuRightFoot , planer->getRightFootPositions());
	VisualizationUtils::generateVisualizationDuplicatesFromTrajectories(_visuLeftFootPositions, _visuLeftFoot, planer->getLeftFootPositions());
	// visualize foot trajectories
	VisualizationUtils::generateVisualizationDuplicatesFromTrajectories(_visuRightFootTrajectory, _visuRightFootwoBorder , planer->getRightFootTrajectory());
	VisualizationUtils::generateVisualizationDuplicatesFromTrajectories(_visuLeftFootTrajectory, _visuLeftFootwoBorder, planer->getLeftFootTrajectory());
}

// get the visualization of the FootVisualization as a SoSeparator Node. The Node will be changed according to Paramater Changes
SoSeparator* FootVisualization::getVisualization() {
	return _visualization;
}

void FootVisualization::showFootPositions(bool isVisible) {
	//_swPositions->whichChild=(isVisible?SO_SWITCH_ALL:SO_SWITCH_NONE);
	if (isVisible)
	{
		std::cout << "Make Foot-Positions visible!" << std::endl;
		_swPositions->whichChild=SO_SWITCH_ALL;
	}
	else
	{
		_swPositions->whichChild=SO_SWITCH_NONE;
	}
}

void FootVisualization::showFootTrajectories(bool isVisible) {
	_swTrajectories->whichChild=(isVisible?SO_SWITCH_ALL:SO_SWITCH_NONE);
}

