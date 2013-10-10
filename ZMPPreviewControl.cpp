#include "ZMPPreviewControl.h"
#include <math.h>
#include <Inventor/nodes/SoSphere.h>
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"

ZMPPreviewControl::ZMPPreviewControl() : _nPreviewCount(150), ZMPPlaner()
{
	std::cout << "Construction ZMP Preview Controller" << std::endl;
}

ZMPPreviewControl::~ZMPPreviewControl()
{
}

void ZMPPreviewControl::setPreview(int N)
{
	_nPreviewCount = N;
	// recalulate?
	computeReference();
}

void ZMPPreviewControl::computeReference()
{
	if (_pPlaner == 0) 
	{
		std::cout << "Cannot plan ZMP without footstep-planer!" << std::endl;
		return;
	}
	std::cout << "Constructing ZMP Preview Reference!" << std::endl;
	bool bLeftSwing = _pPlaner->isStartingWithLeftFoot();
	double dSS = _pPlaner->getSSTime();
	double dDS = _pPlaner->getDSTime();
	int iSS = (int) (dSS*_pPlaner->getSamplesPerSecond());
	int iDS = (int) (dDS*_pPlaner->getSamplesPerSecond());
	_nSamplesDS = iDS;
	_nSamplesSS = iSS;
	Eigen::Matrix3Xf mLeftFoot = _pPlaner->getLeftFootPositions();
	Eigen::Matrix3Xf mRightFoot = _pPlaner->getRightFootPositions();
	// use state-machine for this... state=0 is starting phase, state=1 is walking state, state=2 is active, when walking is coming to an end
	int state = 0;
	// starting with DS-Phase we alternate between DS and SS Phase
	// for every phase we create a reference-ZMP... in SS Phase it is in the center of the standing leg, 
	// in DS-Phase it has to move from previous standing leg to previous swing leg
	// exceptions are the first and last steps... here we have to move the zmp to the standing foot or center of both feet accordingly
	assert(mLeftFoot.cols()==mRightFoot.cols() && "Right and Left Foot Positions have to be the same size!\n");
	int size = mLeftFoot.cols();
	std::cout << "size: " << size << std::endl;
	//int mSize = (size+1)/2 * (iDS+iSS) + iDS;
	int mSize = (size-1) * (iDS+iSS) + iDS;
	std::cout << "Computed Vector Size for refZMP: " << mSize << std::endl;
	_mReference.resize(2, mSize);
	int index=0;
	std::cout << "using following left foot positions: " << std::endl << mLeftFoot << std::endl << std::flush;
	std::cout << "using following right foot positions: " << std::endl << mRightFoot << std::endl << std::flush;
	// premovement phase, state = 0, starting with DS Phase

	// starting DS Phase
	Eigen::Vector2f vLeftFoot;
	Eigen::Vector2f vRightFoot;
	vLeftFoot.setZero();
	vRightFoot.setZero();

	Eigen::Vector2f start;
	Eigen::Vector2f end;

	// initialise with first feet position and move zmp to next standing foot
	vLeftFoot.x() = mLeftFoot.col(0).x();
	vLeftFoot.y() = mLeftFoot.col(0).y();
	vRightFoot.x() = mRightFoot.col(0).x();
	vRightFoot.y() = mRightFoot.col(0).y();


	for (int i=0; i<size; i++) 
	{
		// get FootPositions
		vLeftFoot.x() = mLeftFoot.col(i).x();
		vLeftFoot.y() = mLeftFoot.col(i).y();
		vRightFoot.x() = mRightFoot.col(i).x();
		vRightFoot.y() = mRightFoot.col(i).y();
		std::cout << "Left and right foot: [" << vLeftFoot.x() << "|" << vLeftFoot.y() << "], [" << vRightFoot.x() << "|" << vRightFoot.y() << "]" << std::endl << std::flush;
		// process state
		switch (state) {
		case 0:
			start = (vLeftFoot + vRightFoot) * 0.5f;
			end  = (bLeftSwing?vRightFoot:vLeftFoot);
			//std::cout << "0[DS-Start] Moving from center to standing foot!" << std::endl << start << std::endl << end << std::endl << std::flush;
			//std::cout << "index: " << index << "iDS: " << iDS << std::endl;
			for (int t=0; t<iDS; t++) 
				_mReference.col(index+t)=start+ ((10*(end-start))/pow(iDS,3)*pow(t,3)) + ((15*(start-end))/pow(iDS,4)*pow(t,4)) + ((6*(end-start))/pow(iDS,5)*pow(t,5));
			// increase index
			index += iDS;
			// next is swing-phase
			state = 1;
			break;
		case 1:
			// SS Phase - keep zmp on standing foot
			start = (bLeftSwing?vRightFoot:vLeftFoot);
			//std::cout << "1[SS] Remaining on one foot!" << std::endl << start << std::endl << std::flush;
			//std::cout << "index: " << index << " iSS: " << iSS << " i" << i << std::endl;
			for (int t=0; t<iSS; t++) 
				_mReference.col(index+t)=start;
			// increase index
			index += iSS;
			// normal DS Phase - move from standing foot to swinging foot
			start = (bLeftSwing?vRightFoot:vLeftFoot);
			end = (bLeftSwing?vLeftFoot:vRightFoot);
			//std::cout << "1[DS] Moving from one foot to other foot!" << std::endl << start << std::endl << end << std::endl << std::flush;
			//std::cout << "index: " << index << " iDS: " << iDS << " i" << i << std::endl;
			for (int t=0; t<iDS; t++) 
				_mReference.col(index+t)=start+ ((10*(end-start))/pow(iDS,3)*pow(t,3)) + ((15*(start-end))/pow(iDS,4)*pow(t,4)) + ((6*(end-start))/pow(iDS,5)*pow(t,5));
			// increase index
			index += iDS;
			// switch next swing foot
			bLeftSwing = (bLeftSwing?false:true);
			// move to coming to a halt, when finished
			if (i==size-2)
				state=2;
			break;
		case 2:
			// SS Phase - keep zmp on standing foot
			start = (bLeftSwing?vRightFoot:vLeftFoot);
			//std::cout << "2[SS] Remaining on one foot!" << std::endl << start << std::endl << std::flush;
			//std::cout << "index: " << index << " iSS: " << iSS << " i" << i << std::endl;
			for (int t=0; t<iSS; t++) 
				_mReference.col(index+t)=start;
			// increase index
			index += iSS;
			// ending DS Phase
			start = (bLeftSwing?vRightFoot:vLeftFoot);
			end = (vLeftFoot + vRightFoot) * 0.5f;
			//std::cout << "2[DS-END] Moving from standing foot to center!" << std::endl << start << std::endl << end << std::endl << std::flush;
			//std::cout << "index: " << index << " iDS: " << iDS << " i" << i << std::endl;
			for (int t=0; t<iDS; t++) 
				_mReference.col(index+t)=start+ ((10*(end-start))/pow(iDS,3)*pow(t,3)) + ((15*(start-end))/pow(iDS,4)*pow(t,4)) + ((6*(end-start))/pow(iDS,5)*pow(t,5));
			// increase index
			index += iDS;
			break;
		}
	}
	_bComputed = true;
	buildReferenceVisualization();
	computeCoM();
}

void ZMPPreviewControl::buildReferenceVisualization() 
{
	_referenceNodes->removeAllChildren();
	SoMaterial* colorMat = new SoMaterial();
	colorMat->ambientColor.setValue(0.8,0.1,0.1);
	colorMat->diffuseColor.setValue(0.8,0.1,0.1);
	_referenceNodes->addChild(colorMat);

	SoSeparator* sep = new SoSeparator();
	SoSphere* sp = new SoSphere();
	sp->radius = 0.01;
	
	sep->addChild(sp);

	Eigen::Matrix3Xf positions;
	positions.resize(3, _mReference.cols());
	positions.setZero();
	positions.block(0,0,2,_mReference.cols()) = _mReference;
	std::cout << "Number of entries in position matrix: " << std::endl << positions.cols() << std::endl;

	_pPlaner->generateVisualizationDuplicatesFromTrajectories(_referenceNodes, sep, positions); // (Eigen::Matrix3Xf) positions.block(0,0,3,100));
}

void ZMPPreviewControl::computeCoM()
{
	if (_pPlaner == 0)
	{
		std::cout << "Need Footstep Planer to compute CoM!" << std::endl;
		return;
	}

	std::cout << "Number of samples from LeftFoot trajectory: " << _pPlaner->getLeftFootTrajectory().cols() << std::endl;
	std::cout << "Number of samples from ZMP trajectory: " << _mReference.cols() << std::endl;

	float T = 1.0f / (_pPlaner->getSamplesPerSecond());
	float g = 9.81;
	float hCoM = _pPlaner->getStepHeight();

	Eigen::Matrix3f A;
	Eigen::MatrixXf B;
	Eigen::MatrixXf C;
	
	A.setIdentity();
	A.col(0).x() = T;
	A.col(1).y() = T;
	A.col(2).x() = T*T/2;

	B.resize(3,1);
	B << pow(T,3)/6, pow(T,2)/2, T;

	C.resize(1,3);
	C << 1, 0, -1*hCoM/g;





	computeRealZMP();
}


void ZMPPreviewControl::computeRealZMP() {
}