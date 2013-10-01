#include "FootstepPlaner.h"

#include <VirtualRobot/Robot.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/actions/SoWriteAction.h>

#include "VirtualRobot/CollisionDetection/CollisionChecker.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"


FootstepPlaner::FootstepPlaner(void) : _bChangesMade(false), _bGenerated(false), _pRobot(0)
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

	/*
	_pTranspMaterial = new SoMaterial;
	_pTranspMaterial->ambientColor.setValue(1.0f, 1.0f, 1.0f);
	_pTranspMaterial->emissiveColor.setValue(1.0f, 1.0f, 1.0f);
	_pTranspMaterial->transparency.setValue(0.9f);
	_pTranspMaterial->ref();*/

	//_swTrajectories->addChild(_pTranspMaterial);
	_swTrajectories->addChild(_visuRightFootTrajectory);
	_swTrajectories->addChild(_visuLeftFootTrajectory);
	// can only change between SO_SWITCH_NONE and SO_SWITCH_ALL
}


FootstepPlaner::~FootstepPlaner(void)
{
	/*_visuRightFoot->removeAllChildren();
	_visuRightFootPositions->removeAllChildren();
	_visuRightFootTrajectory->removeAllChildren();
	_visuLeftFoot->removeAllChildren();
	_visuLeftFootPositions->removeAllChildren();
	_visuLeftFootTrajectory->removeAllChildren();*/
	_visualization->removeAllChildren();
	_visuRightFoot->unref();
	_visuLeftFoot->unref();
	_visualization->unref();
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
	// save foot positions
	_vRightFootStart = vRightCenter / 1000.0f;
	_vLeftFootStart = vLeftCenter / 1000.0f;
	// compute walking direction
	Eigen::Vector2f center = (vLeftCenter+vRightCenter)*0.5;
	Eigen::Vector2f centerToLeft = (vLeftCenter - center);
	centerToLeft.normalize();
	Eigen::Matrix2f rotNinety; 
	rotNinety << 0, 1, -1, 0;
	Eigen::Vector2f walkingDirection =  rotNinety * centerToLeft;
	// save rotation for walking
	_mRotateWalking << walkingDirection.x(), centerToLeft.x(), walkingDirection.y(), centerToLeft.y();
	// test the rotation matrix
	walkingDirection.x() = 1.0f;
	walkingDirection.y() = 0.0f;
	walkingDirection = _mRotateWalking * walkingDirection;

	// add an arrow for walking direction
	SoSeparator* sArrow = new SoSeparator();
	SoTranslation* sArrowT = new SoTranslation();
	sArrowT->translation.setValue(center.x()/1000, center.y()/1000, 0);
	sArrow->addChild(sArrowT);
	Eigen::Vector3f walkingDir3D;
	walkingDir3D << walkingDirection.x(), walkingDirection.y(), 0.0f;
	sArrow->addChild (VirtualRobot::CoinVisualizationFactory::CreateArrow(walkingDir3D, 500.0f, 20.0f, 
		VirtualRobot::VisualizationFactory::Color::Blue(0.5f)));
	_visualization->addChild(sArrow);
	// delete old visualizations
	_visuRightFoot->removeAllChildren();
	_visuLeftFoot->removeAllChildren();
	// build visualisation template for each foot (levitated about 2.5f in z-direction for better visibility)
	_visuRightFoot = VirtualRobot::CoinVisualizationFactory::CreateConvexHull2DVisualization(cvRight, plane, 
		VirtualRobot::VisualizationFactory::Color::Blue(), VirtualRobot::VisualizationFactory::Color::Red(),6.0f,Eigen::Vector3f(0,0,2.5f));
	_visuLeftFoot = VirtualRobot::CoinVisualizationFactory::CreateConvexHull2DVisualization(cvLeft, plane, 
		VirtualRobot::VisualizationFactory::Color::Green(), VirtualRobot::VisualizationFactory::Color::Red(),6.0f,Eigen::Vector3f(0,0,2.5f));
	// ref(), otherwise the Shapes get deleted!
	_visuRightFoot->ref();
	_visuLeftFoot->ref();
	// build a different representation of the feet shape for trajectory visualization
	_visuRightFootwoBorder = VirtualRobot::CoinVisualizationFactory::CreateConvexHull2DVisualization(cvRight, plane, 
		VirtualRobot::VisualizationFactory::Color::Blue(0.9f), VirtualRobot::VisualizationFactory::Color::Gray(),0.1f,Eigen::Vector3f(0,0,2.5f));
	_visuLeftFootwoBorder = VirtualRobot::CoinVisualizationFactory::CreateConvexHull2DVisualization(cvLeft, plane, 
		VirtualRobot::VisualizationFactory::Color::Green(0.9f), VirtualRobot::VisualizationFactory::Color::Gray(),0.1f,Eigen::Vector3f(0,0,2.5f));
	_visuRightFootwoBorder->ref();
	_visuLeftFootwoBorder->ref();	
	// build the visualization
	std::cout << "Shape of Feet computed!" << std::endl;
	buildVisualization();
}

// build visualization for feet position and feet trajectories
void FootstepPlaner::buildVisualization() {
	if (!_bGenerated) {
		std::cout << "Cannot build visualization, because the trajectories were not yet generated!" << std::endl;
		return;
	}

	// clean up old visualizations
	//* TODO: enable this later again
	std::cout << "Number of Children of _visuRightFootPositions before: " << _visuRightFootPositions->getNumChildren();
	_visuRightFootPositions->removeAllChildren();
	_visuLeftFootPositions->removeAllChildren();
	_visuRightFootTrajectory->removeAllChildren();
	_visuLeftFootTrajectory->removeAllChildren();
	//_visuLeftFootTrajectory->addChild(_pTranspMaterial);
	//_visuRightFootTrajectory->addChild(_pTranspMaterial);
	std::cout << ", and after: " << _visuRightFootPositions->getNumChildren() << std::endl;

	//_visualization->addChild(_visuLeftFoot);
	// visualize foot positions
	/*SoSeparator* spSep = new SoSeparator();
	SoSphere* sp = new SoSphere();
	sp->radius = 0.02;
	spSep->addChild(sp);*/
	//generateVisualizationDuplicatesFromTrajectories(_visuRightFootPositions, spSep /*_visuRightFoot*/ , _mRFootPositions);
	generateVisualizationDuplicatesFromTrajectories(_visuRightFootPositions, _visuRightFoot , _mRFootPositions);
	generateVisualizationDuplicatesFromTrajectories(_visuLeftFootPositions, _visuLeftFoot, _mLFootPositions);
	// visualize foot trajectories
	generateVisualizationDuplicatesFromTrajectories(_visuRightFootTrajectory, _visuRightFootwoBorder , _mRFootTrajectory);
	generateVisualizationDuplicatesFromTrajectories(_visuLeftFootTrajectory, _visuLeftFootwoBorder, _mLFootTrajectory);
}

// adds the visualization of the FootstepPlaner to a given SoSeparatorNode
SoSeparator* FootstepPlaner::getVisualization() {
	// TODO: check if computations are finished
	return _visualization;
}

void FootstepPlaner::showFootPositions(bool isVisible) {
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

void FootstepPlaner::showFootTrajectories(bool isVisible) {
	_swTrajectories->whichChild=(isVisible?SO_SWITCH_ALL:SO_SWITCH_NONE);
}

/*
 * This method can generate a trajectory of a certain object in a given SoSeparator-Node
 * The Object is inserted according to the translation values given in the Matrix3d
 * 
 */
void FootstepPlaner::generateVisualizationDuplicatesFromTrajectories(SoSeparator* whereToInsert, SoSeparator* whatToInsert, Eigen::Matrix3Xd &whereToTranslate) {
	Eigen::Vector3d pos, prev;
	pos.setZero();
	prev.setZero();
	for (int i=0; i<whereToTranslate.cols(); i++)
	{
		prev = pos;
		pos = whereToTranslate.col(i);
		// if consecutive positions are the same, display the item just once
		if ((i!=0) && (prev==pos))
			continue;
		SoSeparator *pSep = new SoSeparator();
		SoTranslation *pTranslate = new SoTranslation();
		pTranslate->translation.setValue(pos(0), pos(1), pos(2));
		whereToInsert->addChild(pSep);
		pSep->addChild(pTranslate);
		// TODO: add a node for transparency
		pSep->addChild(whatToInsert);
		//whatToInsert->ref();
	}
}


/*
 * Save as scene Graph to a test file
 */

void FootstepPlaner::writeSceneGraphToFile(SoSeparator* node) 
{
	SoOutput* output = new SoOutput();
	output->openFile("sceneGraphDebug.txt");
	SoWriteAction* pWrite = new SoWriteAction(output);
	pWrite->apply(node);
	//node->write(pWrite);
	std::cout << "Scene Graph saved to file!" << std::endl;
}