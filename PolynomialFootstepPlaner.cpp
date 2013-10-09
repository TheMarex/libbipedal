#include "PolynomialFootstepPlaner.h"
#include <VirtualRobot/Robot.h>

#include "VirtualRobot/CollisionDetection/CollisionChecker.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"

PolynomialFootstepPlaner::PolynomialFootstepPlaner(void) : _dStepLength(0.3f), _dStepWidth(0.2f), _dStepHeight(0.3f), _dStepPeriod(0.8f), _dSingleSupportPhase(0.7f), _dDoubleSupportPhase(0.1f), _iSampleSize(100), _bLeftFootFirst(true)
{
} 

PolynomialFootstepPlaner::~PolynomialFootstepPlaner(void) {
}

void PolynomialFootstepPlaner::setParameters(double stepLength, double stepPeriod, double doubleSupportPhase, double stepHeight, int sampleSize) {
	if (_bParametersInitialized) {
		//TODO: first delete all array, vectors and matrices
	}
	
	_dStepLength = stepLength;
	_dStepPeriod = stepPeriod;
	_dDoubleSupportPhase = doubleSupportPhase;
	_dSingleSupportPhase = _dStepPeriod - _dDoubleSupportPhase;
	_dStepHeight = stepHeight;
	_iSampleSize = sampleSize;
	_bParametersInitialized = true;
	
	generate();
}

// start walking with left foot first
void PolynomialFootstepPlaner::setLeftFootFirst() 
{
	_bLeftFootFirst=true;
}

// start walking with right foot first
void PolynomialFootstepPlaner::setRightFootFirst()
{
	_bLeftFootFirst=false;
}

/*
// TODO:
SoSeparator* PolynomialFootstepPlaner::getShapeLeftFoot() {
	return 0;
}
*/

/*
// TODO:
SoSeparator* PolynomialFootstepPlaner::getShapeRightFoot() {
	return 0;
}
*/

void PolynomialFootstepPlaner::generate(int numberOfSteps) {
	if (numberOfSteps<2) 
		numberOfSteps=2;
	_iNumberOfSteps = numberOfSteps;
	_mLFootPositions = Eigen::Matrix3Xf::Zero(3, _iNumberOfSteps+1);
	_mRFootPositions = Eigen::Matrix3Xf::Zero(3, _iNumberOfSteps+1);
	int stepCounter=0;
	// ***************************************************
	// ** calculate generalized swinging leg trajectory **
	// ***************************************************
	int iSamplesPerStep = (int) (_iSampleSize * _dStepPeriod)+1;
	double sampleDelta = 1.0f / _iSampleSize;
	// initialise Matrix
	_footTrajectory = Eigen::Matrix3Xf::Zero(3, iSamplesPerStep);
	_footTrajectoryFirstLast = Eigen::Matrix3Xf::Zero(3, iSamplesPerStep);
	// percentage of different Phases
	_dSS = (double) (_dSingleSupportPhase / _dStepPeriod);
	_dDS = (double) (_dDoubleSupportPhase / _dStepPeriod);
	// total number of samples for each phase
	int iSS = (int) (iSamplesPerStep * _dSS);
	int iDS = (int) (iSamplesPerStep * _dDS);
	// temporary variables
	double T1 = _dSingleSupportPhase;
	double T2 = T1*T1;
	double T3 = T2*T1;
	double T4 = T3*T1;
	double S = _dStepLength;
	double H = _dStepHeight;
	double cx, dx, bz, cz, dz, xtemp, ztemp, x1, x2, x3, x4;
	// calculating foot trajectory for swinging leg including resting phase
	for (int i=0; i<_footTrajectory.cols(); i++) {
		// precalculate {c*x, d*x, b*z, c*z, d*z, x^1, x^2, x^3, x^4} for optimization purposes
		cx = -2*S/T3;
		dx = 3*S/T2;
		bz = 16*H/T4;
		cz = -32*H/T3;
		dz = 16*H/T2;
		x1 = sampleDelta*i;
		x2 = x1*x1;
		x3 = x2*x1;
		x4 = x3*x1;
		if (i<iSS) 
		{
			// swinging phase
			xtemp =       cx*x3+dx*x2;
			ztemp = bz*x4+cz*x3+dz*x2;
		} else {
			// resting phase continues with last values
			xtemp = S;
			ztemp = 0;
		}
		_footTrajectory(0, i)= xtemp;
		_footTrajectory(2, i)= ztemp;
		// half steps are half the length and half the height
		_footTrajectoryFirstLast(0, i) = xtemp/2;
		_footTrajectoryFirstLast(2, i) = ztemp/2;
	}
	// ******************************************
	// ** calculate Foot Positions for n-Steps **
	// ******************************************
	// initialise Matrices
	int iSamples = iSamplesPerStep * _iNumberOfSteps + iDS*2;
	_mLFootTrajectory =  Eigen::Matrix3Xf::Zero(3, iSamples);
	_mRFootTrajectory =  Eigen::Matrix3Xf::Zero(3, iSamples);
	bool bLeft = (_bLeftFootFirst?true:false);
	bool halfStep = true;
	int index = 0;
	// determine starting positions of left and right foot
	Eigen::Vector3f vLeftFoot;
	Eigen::Vector3f vRightFoot;
	Eigen::Vector3f vTempL, vTempR;
	vLeftFoot.setZero();
	vRightFoot.setZero();
	vLeftFoot(1) = -_dStepWidth/2;
	vRightFoot(1) = _dStepWidth/2;
	_vDeltaLeftFoot = vLeftFoot;
	_vDeltaRightFoot = vRightFoot;

	_mLFootPositions.col(stepCounter)=vLeftFoot;
	_mRFootPositions.col(stepCounter)=vRightFoot;
	// starting with full DS-Phase
	for (int j=0; j<iDS; j++) {
		_mLFootTrajectory.col(index) = vLeftFoot;
		_mRFootTrajectory.col(index) = vRightFoot;
		index++;
	}
	for (int i=0; i<_iNumberOfSteps; i++) {
		// starting and ending steps are half steps, the other steps are full steps
		if ((i==0) || (i==_iNumberOfSteps-1))
			halfStep = true;
		else
			halfStep = false;
		// TODO: elegantere Lösung implementieren
		vTempL.setZero();
		vTempR.setZero();
		if (halfStep) {
			for (int j=0; j<iSamplesPerStep; j++) {
				// which foot to move?
				if (bLeft)
					vTempL = _footTrajectoryFirstLast.col(j);
				else
					vTempR = _footTrajectoryFirstLast.col(j);
				_mLFootTrajectory.col(index) = vLeftFoot + vTempL;
				_mRFootTrajectory.col(index) = vRightFoot + vTempR;
				index++;
			}
		} else {
			for (int j=0; j<iSamplesPerStep; j++) {
				// which foot to move?
				if (bLeft) 
					vTempL = _footTrajectory.col(j);
				else
					vTempR = _footTrajectory.col(j);
				_mLFootTrajectory.col(index) = vLeftFoot + vTempL;
				_mRFootTrajectory.col(index) = vRightFoot + vTempR;
				index++;
			}
		}
		// switch feet
		bLeft = (bLeft?false:true);
		// save new foot positions to temporary vectors
		vLeftFoot = _mLFootTrajectory.col(index-1);
		vRightFoot = _mRFootTrajectory.col(index-1);
		stepCounter++;
		_mLFootPositions.col(stepCounter)=vLeftFoot;
		_mRFootPositions.col(stepCounter)=vRightFoot;

	}
	// insert ending DS-phase
	for (int j=0; j<iDS; j++) {
		_mLFootTrajectory.col(index) = vLeftFoot;
		_mRFootTrajectory.col(index) = vRightFoot;
		index++;
	}
	_bGenerated = true;
	// **********************************
	// ** generate visualization nodes **
	// **********************************
	buildVisualization();
}


