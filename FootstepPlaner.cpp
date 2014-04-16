#include "FootstepPlaner.h"

#include <VirtualRobot/Robot.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/actions/SoWriteAction.h>

#include "VirtualRobot/CollisionDetection/CollisionChecker.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"


FootstepPlaner::FootstepPlaner(void) : _dStepLength(0.3f), _dStepWidth(0.2f), _dStepHeight(0.3f), _dStepPeriod(0.8f), _dCoMHeight(0.75f), _dSingleSupportPhase(0.7f), _dDoubleSupportPhase(0.1f), _iSampleSize(100), _bLeftFootFirst(true), _bChangesMade(false), _bGenerated(false)
{
    _mInitialCoMPosition.setZero();
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
	_vRightFootCenter = vRightCenter / 1000.0f;
	_vLeftFootCenter = vLeftCenter / 1000.0f;
	// translate points of FootShape so, that center of convex hull is (0|0)
	for (int i=0; i<cvRight->vertices.size(); i++)
		cvRight->vertices[i] -= vRightCenter;
	for (int i=0; i<cvLeft->vertices.size(); i++)
		cvLeft->vertices[i] -= vLeftCenter;
	// compute walking direction
	Eigen::Vector2f center = (vLeftCenter+vRightCenter)*0.5;
	Eigen::Vector2f centerToLeft = (vLeftCenter - center);
	_vRobotCenter = center * 0.001f;
	centerToLeft.normalize();
	Eigen::Matrix2f rotNinety; 
	rotNinety << 0, 1, -1, 0;
	Eigen::Vector2f walkingDirection =  rotNinety * centerToLeft;
	// save rotation for walking
	_mRotateWalking << walkingDirection.x(), centerToLeft.x(), walkingDirection.y(), centerToLeft.y();
	// test the rotation matrix
	/*
	walkingDirection.x() = 1.0f;
	walkingDirection.y() = 0.0f;
	walkingDirection = _mRotateWalking * walkingDirection;
	*/
	// add an arrow for walking direction
	SoSeparator* sArrow = new SoSeparator();
	SoTranslation* sArrowT = new SoTranslation();
	sArrowT->translation.setValue(center.x()/1000, center.y()/1000, 1.0f);
	sArrow->addChild(sArrowT);
	Eigen::Vector3f walkingDir3D;
	walkingDir3D << walkingDirection.x(), walkingDirection.y(), 0.0f;
	sArrow->addChild (VirtualRobot::CoinVisualizationFactory::CreateArrow(walkingDir3D, 500.0f, 20.0f, 
		VirtualRobot::VisualizationFactory::Color::Blue(0.5f)));
    //_visualization->addChild(sArrow);
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

	// rotate generated walking pattern to walking direction
	Eigen::Matrix3f rot;
	rot.setIdentity();
	rot.block(0,0,2,2) = _mRotateWalking;
	Eigen::Matrix3Xf rFootPositions = rot * _mRFootPositions;
	Eigen::Matrix3Xf lFootPositions = rot * _mLFootPositions;
	Eigen::Matrix3Xf rFootTrajectories = rot * _mRFootTrajectory;
	Eigen::Matrix3Xf lFootTrajectories = rot * _mLFootTrajectory;

	// translate foot positions so that inital foot position matches actual position
	Eigen::Vector3f rDelta, lDelta;
	rDelta.setZero();
	lDelta.setZero();

    rDelta.x() =  - rFootPositions.col(0).x() + _vRightFootCenter.x(); //- _vRobotCenterStart.x();
	rDelta.y() =  - rFootPositions.col(0).y() + _vRightFootCenter.y(); // - _vRobotCenterStart.x();  +_vRightFootCenter.y()
    // rDelta = _vRightFootCenter-rFootPositions.block(0,0,2,1);
	lDelta.x() =  - lFootPositions.col(0).x() + _vLeftFootCenter.x(); //- _vRobotCenterStart.x();
	lDelta.y() =  - lFootPositions.col(0).y() + _vLeftFootCenter.y(); // - _vRobotCenterStart.x();  +_vRightFootCenter.y()
    // lDelta = _vLeftFootCenter-lFootPositions.block(0,0,2,1);
	for (int i=0; i<rFootPositions.cols(); i++)
		rFootPositions.col(i) += rDelta ;
	for (int i=0; i<lFootPositions.cols(); i++)
		lFootPositions.col(i) += lDelta ;
	for (int i=0; i<rFootTrajectories.cols(); i++)
		rFootTrajectories.col(i) += rDelta ;
	for (int i=0; i<lFootTrajectories.cols(); i++)
		lFootTrajectories.col(i) += lDelta ;
	
	// visualize foot positions
	generateVisualizationDuplicatesFromTrajectories(_visuRightFootPositions, _visuRightFoot , rFootPositions);
	generateVisualizationDuplicatesFromTrajectories(_visuLeftFootPositions, _visuLeftFoot, lFootPositions);
	// visualize foot trajectories
	generateVisualizationDuplicatesFromTrajectories(_visuRightFootTrajectory, _visuRightFootwoBorder , rFootTrajectories);
	generateVisualizationDuplicatesFromTrajectories(_visuLeftFootTrajectory, _visuLeftFootwoBorder, lFootTrajectories);
	// save transformed foot positions
	_mRFootPositionsTransformed.resize(3, rFootPositions.cols());
	_mLFootPositionsTransformed.resize(3, lFootPositions.cols());
	_mRFootPositionsTransformed = rFootPositions; //.block(0,0,3,rFootPositions.cols());
	_mLFootPositionsTransformed = lFootPositions; //.block(0,0,3,lFootPositions.cols());
	// save transformed feet trajectory
	_mLFootTrajectoryTransformed.resize(3, lFootTrajectories.cols());
	_mRFootTrajectoryTransformed.resize(3, rFootTrajectories.cols());
	_mLFootTrajectoryTransformed = lFootTrajectories;
	_mRFootTrajectoryTransformed = rFootTrajectories;
}

// get the visualization of the FootstepPlaner as a SoSeparator Node. The Node will be changed according to Paramater Changes
SoSeparator* FootstepPlaner::getVisualization() {
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
void FootstepPlaner::generateVisualizationDuplicatesFromTrajectories(SoSeparator* whereToInsert, SoSeparator* whatToInsert, Eigen::Matrix3Xf &whereToTranslate) {
	Eigen::Vector3f pos, prev;
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
		pTranslate->translation.setValue(pos.x(), pos.y(), pos.z());
		whereToInsert->addChild(pSep);
		pSep->addChild(pTranslate);
		// TODO: add a node for transparency
		pSep->addChild(whatToInsert);
		//whatToInsert->ref();
	}
}

void FootstepPlaner::generateVisualizationForLineTrajectories(SoSeparator* whereToInsert, Eigen::Matrix3Xf positionList, float rColor, float gColor, float bColor, float scale)
{
    Eigen::Matrix4f mFrom, mTo;
    mFrom.setIdentity();
    mTo.setIdentity();
    for (int i=0; i<positionList.cols()-1; i++) {
        mFrom.block(0,3,3,1) = positionList.col(i)*scale;
        mTo.block(0,3,3,1) = positionList.col(i+1)*scale;
        //VirtualRobot::VisualizationNodePtr p = visualizationFactory->createLine(vFrom, vTo, 2.0f, 0.1f, 0.8f, 0.1f);
        //_realNodes->addChild(VirtualRobot::CoinVisualizationFactory::getCoinVisualization(p));
        whereToInsert->addChild(VirtualRobot::CoinVisualizationFactory::createCoinLine(mFrom, mTo, 5.0f, rColor, gColor, bColor));
    }

}

void FootstepPlaner::generateVisualizationForLineTrajectories(SoSeparator* whereToInsert, Eigen::Vector3f from, Eigen::Vector3f to, float rColor, float gColor, float bColor,  float scale)
{
    Eigen::Matrix4f mFrom, mTo;
    mFrom.setIdentity();
    mTo.setIdentity();
    mFrom.block(0,3,3,1) = from*scale;
    mTo.block(0,3,3,1) = to*scale;
    whereToInsert->addChild(VirtualRobot::CoinVisualizationFactory::createCoinLine(mFrom, mTo, 5.0f, rColor, gColor, bColor));
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


Eigen::Matrix3Xf const FootstepPlaner::getLeftFootPositions()
{
	return _mLFootPositionsTransformed;
}

Eigen::Matrix3Xf const FootstepPlaner::getRightFootPositions() 
{
	return _mRFootPositionsTransformed;
}


Eigen::Matrix3Xf const FootstepPlaner::getLeftFootTrajectory() 
{
	return _mLFootTrajectoryTransformed;
}

Eigen::Matrix3Xf const FootstepPlaner::getRightFootTrajectory()
{
	return _mRFootTrajectoryTransformed;
}

