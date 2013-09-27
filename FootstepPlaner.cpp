#include "FootstepPlaner.h"

#include <VirtualRobot/Robot.h>
#include <Inventor/nodes/SoTranslation.h>


#include "VirtualRobot/CollisionDetection/CollisionChecker.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"


FootstepPlaner::FootstepPlaner(void) : _bChangesMade(false), _bGenerated(false), _pRobot(0)
{
	_visuRightFoot = new SoSeparator();
	_visuRightFootPositions  = new SoSeparator();
	_visuRightFootTrajectory = new SoSeparator();
	_visuLeftFoot = new SoSeparator();
	_visuLeftFootPositions = new SoSeparator();
	_visuLeftFootTrajectory = new SoSeparator();
	_visualization = new SoSeparator();
}


FootstepPlaner::~FootstepPlaner(void)
{
	_visuRightFoot->removeAllChildren();
	_visuRightFootPositions->removeAllChildren();
	_visuRightFootTrajectory->removeAllChildren();
	_visuLeftFoot->removeAllChildren();
	_visuLeftFootPositions->removeAllChildren();
	_visuLeftFootTrajectory->removeAllChildren();
	_visualization->removeAllChildren();
}

// assign a Robot Model to the Footstep Planer
void FootstepPlaner::setRobotModel(VirtualRobot::RobotPtr pRobot, std::string nameRightFoot, std::string nameLeftFoot) {
	_pRobot = pRobot;
	_sRightFootName = nameRightFoot;
	_sLeftFootName = nameLeftFoot;
	// compute shape of feet (support polygon of left and right foot)
	computeFeetShape();
}

// compute the shape of each foot and generate a visualization for every foot
void FootstepPlaner::computeFeetShape() {
	// check for robot model
	if (_pRobot == 0) {
		std::cout << "Cannot compute feet shape, because no robot model is loaded!" << std::endl;
		return;
	}
	// get the Floor
	VirtualRobot::MathTools::Plane plane =  VirtualRobot::MathTools::getFloorPlane();
	// get the Robot Nodes for the Feet
	VirtualRobot::RobotNodePtr pRightFoot = _pRobot->getRobotNode(_sRightFootName);
	VirtualRobot::RobotNodePtr pLeftFoot = _pRobot->getRobotNode(_sLeftFootName);
	if ((pRightFoot==0) || (pLeftFoot==0))
	{
		std::cout << "Cannot compute feet shape, because the feet where not found!" << std::endl;
		return;
	}
	// get collision model of the feet
	VirtualRobot::CollisionModelPtr pRightCollision = pRightFoot->getCollisionModel();
	VirtualRobot::CollisionModelPtr pLeftCollision = pLeftFoot->getCollisionModel();
	if ((pRightCollision==0) || (pLeftCollision==0))
	{
		std::cout << "Cannot compute feet shape, because the collision shapes of the feet could not be retrieved!" << std::endl;
		return;
	}
	// let the feet collide with the floor and get the collision points
	VirtualRobot::CollisionCheckerPtr colChecker = VirtualRobot::CollisionChecker::getGlobalCollisionChecker();
	std::vector< VirtualRobot::MathTools::ContactPoint > pointsRightFoot;
	std::vector< VirtualRobot::MathTools::ContactPoint > pointsLeftFoot;
	std::vector< Eigen::Vector2f > pointsRight2D;
	std::vector< Eigen::Vector2f > pointsLeft2D;
	// let the collision begin
	colChecker->getContacts(plane, pRightCollision, pointsRightFoot, 5.0f);
	colChecker->getContacts(plane, pLeftCollision, pointsLeftFoot, 5.0f);
	// project the points on the floor
	for (size_t u=0;u<pointsRightFoot.size();u++)
	{
		Eigen::Vector2f pt2d = VirtualRobot::MathTools::projectPointToPlane2D(pointsRightFoot[u].p,plane);
		pointsRight2D.push_back(pt2d);
	}
	for (size_t u=0;u<pointsLeftFoot.size();u++)
	{
		Eigen::Vector2f pt2d = VirtualRobot::MathTools::projectPointToPlane2D(pointsLeftFoot[u].p,plane);
		pointsLeft2D.push_back(pt2d);
	}
	// calculate the convex hulls and the appropriate centers
	VirtualRobot::MathTools::ConvexHull2DPtr cvRight = VirtualRobot::MathTools::createConvexHull2D(pointsRight2D);
	VirtualRobot::MathTools::ConvexHull2DPtr cvLeft = VirtualRobot::MathTools::createConvexHull2D(pointsLeft2D);
	Eigen::Vector2f vRightCenter = VirtualRobot::MathTools::getConvexHullCenter(cvRight);
	Eigen::Vector2f vLeftCenter = VirtualRobot::MathTools::getConvexHullCenter(cvLeft);
	// TODO: save foot positions and compute walking direction
	
	// delete old visualizations
	_visuLeftFoot->removeAllChildren();
	_visuRightFoot->removeAllChildren();
	// build visualisation template for each foot (levitated about 2.5f in z-direction for better visibility)
	_visuRightFoot = VirtualRobot::CoinVisualizationFactory::CreateConvexHull2DVisualization(cvRight, plane, 
		VirtualRobot::VisualizationFactory::Color::Blue(), VirtualRobot::VisualizationFactory::Color::Red(),6.0f,Eigen::Vector3f(0,0,2.5f));
	_visuLeftFoot = VirtualRobot::CoinVisualizationFactory::CreateConvexHull2DVisualization(cvLeft, plane, 
		VirtualRobot::VisualizationFactory::Color::Green(), VirtualRobot::VisualizationFactory::Color::Red(),6.0f,Eigen::Vector3f(0,0,2.5f));
	// TODO: build a different representation of the feet shape for trajectory visualization
	// build the visualization
	buildVisualization();
}

// build visualization for feet position and feet trajectories
void FootstepPlaner::buildVisualization() {
	if (!_bGenerated) {
		std::cout << "Cannot build visualization, because the trajectories were not yet generated!" << std::endl;
		return;
	}
	//Eigen::Matrix3Xd mRightFootPositions = _mRFootPositions;
	//Eigen::Matrix3Xd mLeftFootPositions = _mLFootPositions;
	// clean up old visualizations
	

	/*_visuRightFootPositions->removeAllChildren();
	_visuLeftFootPositions->removeAllChildren();
	_visuRightFootTrajectory->removeAllChildren();
	_visuLeftFootTrajectory->removeAllChildren();*/
	
	
	// visualize foot positions
	generateVisualizationDuplicatesFromTrajectories(_visuRightFootPositions, _visuRightFoot , _mRFootPositions);
	generateVisualizationDuplicatesFromTrajectories(_visuLeftFootPositions, _visuLeftFoot, _mLFootPositions);
	// visualize foot trajectories
	generateVisualizationDuplicatesFromTrajectories(_visuRightFootTrajectory, _visuRightFoot , _mRFootTrajectory);
	generateVisualizationDuplicatesFromTrajectories(_visuLeftFootTrajectory, _visuLeftFoot, _mLFootTrajectory);



	/*
	Eigen::Vector3d pos, prev;
	for (int i=0; i<mRightFootPositions.cols(); i++)
	{
		prev = pos;
		pos = mRightFootPositions.col(i);
		SoSeparator *pSep = new SoSeparator();
		SoTranslation *pTranslate = new SoTranslation();
		// display every foot position just once
		if ((i!=0) && (prev==pos))
			continue;
		// TODO: rotation and translation depending on walking direction and feet positions
		pTranslate->translation.setValue(pos(0), pos(1), pos(2));
		_visuRightFootPositions->addChild(pSep);
		pSep->addChild(pTranslate);
		pSep->addChild(_visuRightFoot);
	}
	* /
	for (int i=0; i<mLeftFootPositions.cols(); i++)
	{
		prev = pos;
		pos = mLeftFootPositions.col(i);
		SoSeparator *pSep = new SoSeparator();
		SoTranslation *pTranslate = new SoTranslation();
		// display every foot position just once
		if ((i!=0) && (prev==pos))
			continue;
		// TODO: rotation and translation depending on walking direction and feet positions
		pTranslate->translation.setValue(pos(0), pos(1), pos(2));
		_visuLeftFootPositions->addChild(pSep);
		pSep->addChild(pTranslate);
		pSep->addChild(_visuLeftFoot);
	}*/

}

// adds the visualization of the FootstepPlaner to a given SoSeparatorNode
void FootstepPlaner::addToVisualization(SoSeparator* root) {
	// TODO: check if computations are finished
	root->addChild(_visuLeftFootPositions);
	root->addChild(_visuRightFootPositions);
	root->addChild(_visuLeftFootTrajectory);
	root->addChild(_visuRightFootTrajectory);}


/*
 * This method can generate a trajectory of a certain object in a given SoSeparator-Node
 * The Object is inserted according to the translation values given in the Matrix3d
 * 
 */
void FootstepPlaner::generateVisualizationDuplicatesFromTrajectories(SoSeparator* whereToInsert, SoSeparator* whatToInsert, Eigen::Matrix3Xd &whereToTranslate) {
	Eigen::Vector3d pos, prev;
	for (int i=0; i<whereToTranslate.cols(); i++)
	{
		prev = pos;
		pos = whereToTranslate.col(i);
		SoSeparator *pSep = new SoSeparator();
		SoTranslation *pTranslate = new SoTranslation();
		// if consecutive positions are the same, display the item just once
		if ((i!=0) && (prev==pos))
			continue;
		pTranslate->translation.setValue(pos(0), pos(1), pos(2));
		whereToInsert->addChild(pSep);
		pSep->addChild(pTranslate);
		// TODO: add a node for transparency
		pSep->addChild(whatToInsert);
	}
}