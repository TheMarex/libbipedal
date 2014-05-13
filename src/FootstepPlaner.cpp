#include "FootstepPlaner.h"

#include <VirtualRobot/Robot.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/actions/SoWriteAction.h>

#include "VirtualRobot/CollisionDetection/CollisionChecker.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"

#include <boost/assert.hpp>


FootstepPlaner::FootstepPlaner(void)
: _dStepLength(0.3f)
, _dStepWidth(0.2f)
, _dStepHeight(0.3f)
, _dStepPeriod(0.8f)
, _dCoMHeight(0.75f)
, _dSingleSupportPhase(0.7f)
, _dDoubleSupportPhase(0.1f)
, _iSampleSize(50)
, _bLeftFootFirst(true)
, _bChangesMade(false),
_bGenerated(false)
{
    _mInitialCoMPosition.setZero();
}

void FootstepPlaner::generate(int numberOfSteps)
{
	computeFeetTrajectories(numberOfSteps);
	transformFootPositions();
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
}

// build visualization for feet position and feet trajectories
void FootstepPlaner::transformFootPositions() {
	BOOST_ASSERT(_bGenerated);

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

const Eigen::Matrix3Xf& FootstepPlaner::getLeftFootPositions()
{
	return _mLFootPositionsTransformed;
}

const Eigen::Matrix3Xf& FootstepPlaner::getRightFootPositions() 
{
	return _mRFootPositionsTransformed;
}


const Eigen::Matrix3Xf& FootstepPlaner::getLeftFootTrajectory() 
{
	return _mLFootTrajectoryTransformed;
}

const Eigen::Matrix3Xf& FootstepPlaner::getRightFootTrajectory()
{
	return _mRFootTrajectoryTransformed;
}

