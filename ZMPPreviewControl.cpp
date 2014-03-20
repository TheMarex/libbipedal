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


// computes the ZMP-Reference from Foot-Positions
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
    assert(mLeftFoot.cols()==mRightFoot.cols() && "Right and Left Foot Position-Matrices have to be the same size!\n");
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
        case 0: // starting phase
            // 1. starting, move ZMP from center of both feet to next standing foot
			start = (vLeftFoot + vRightFoot) * 0.5f;
			end  = (bLeftSwing?vRightFoot:vLeftFoot);
            //std::cout << "0[DS-Start] Moving from center to standing foot!" << std::endl << start << std::endl << end << std::endl << std::flush;
            std::cout << "0[DS-Start][ZMPPreview] index: " << index << "iDS: " << iDS << std::endl;
			for (int t=0; t<iDS; t++) 
				_mReference.col(index+t)=start+ ((10*(end-start))/pow(iDS,3)*pow(t,3)) + ((15*(start-end))/pow(iDS,4)*pow(t,4)) + ((6*(end-start))/pow(iDS,5)*pow(t,5));
			// increase index
			index += iDS;
			// next is swing-phase
			state = 1;
			break;
        case 1: // walking phase
            // 1. SS Phase - keep zmp on standing foot
			start = (bLeftSwing?vRightFoot:vLeftFoot);
			//std::cout << "1[SS] Remaining on one foot!" << std::endl << start << std::endl << std::flush;
            std::cout << "1[SS][ZMPPreview] index: " << index << " iSS: " << iSS << " i" << i << std::endl;
			for (int t=0; t<iSS; t++) 
				_mReference.col(index+t)=start;
			// increase index
			index += iSS;
            // 2. normal DS Phase - move from standing foot to swinging foot
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
        case 2: // stopping phase
            // 1. SS Phase - keep zmp on standing foot
			start = (bLeftSwing?vRightFoot:vLeftFoot);
			//std::cout << "2[SS] Remaining on one foot!" << std::endl << start << std::endl << std::flush;
            std::cout << "2[SS][ZMPPreview] index: " << index << " iSS: " << iSS << " i" << i << std::endl;
			for (int t=0; t<iSS; t++) 
				_mReference.col(index+t)=start;
			// increase index
			index += iSS;
            // 2. ending DS Phase
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
    std::cout << "[endfor][ZMPPreview] index: " << index << std::endl;
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
	sp->radius = 0.002;
	sep->addChild(sp);

	Eigen::Matrix3Xf positions;
	positions.resize(3, _mReference.cols());
	positions.setZero();
	positions.block(0,0,2,_mReference.cols()) = _mReference;
	std::cout << "Number of entries in position matrix: " << std::endl << positions.cols() << std::endl;

	_pPlaner->generateVisualizationDuplicatesFromTrajectories(_referenceNodes, sep, positions); // (Eigen::Matrix3Xf) positions.block(0,0,3,100));
}

// Visualize the real-ZMP computed from CoM trajectory ...  TODO: check again
void ZMPPreviewControl::buildRealZMPVisualization()
{
    _realNodes->removeAllChildren();
    SoMaterial* colorMat = new SoMaterial();
    colorMat->ambientColor.setValue(0.1,0.8,0.1);
    colorMat->diffuseColor.setValue(0.1,0.8,0.1);
    _realNodes->addChild(colorMat);

    SoSeparator* sep = new SoSeparator();
    SoSphere* sp = new SoSphere();
    sp->radius = 0.002;

    sep->addChild(sp);

    Eigen::Matrix3Xf positions;
    positions.resize(3, _mZMP.cols());
    positions.setZero();
    positions.block(0,0,2,_mZMP.cols()) = _mZMP;
    std::cout << "Number of entries in realZMP-position matrix: " << std::endl << positions.cols() << std::endl;

    _pPlaner->generateVisualizationDuplicatesFromTrajectories(_realNodes, sep, positions); // (Eigen::Matrix3Xf) positions.block(0,0,3,100));
}

// Visualize the CoM trajectory computed from reference ZMP - TODO: check for error
void ZMPPreviewControl::buildCoMVisualization()
{
    _comNodes->removeAllChildren();
    SoMaterial* colorMat = new SoMaterial();
    colorMat->ambientColor.setValue(0.1,0.1,0.8);
    colorMat->diffuseColor.setValue(0.1,0.1,0.8);
    _comNodes->addChild(colorMat);

    SoSeparator* sep = new SoSeparator();
    SoSphere* sp = new SoSphere();
    sp->radius = 0.01;

    sep->addChild(sp);
    std::cout << "using following 10 com coordinates: " << std::endl << _mCoM.block(0,0,3,10) << std::endl << std::flush;

    _pPlaner->generateVisualizationDuplicatesFromTrajectories(_comNodes, sep, _mCoM); // (Eigen::Matrix3Xf) positions.block(0,0,3,100));
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
    std::cout << "Number of samples in computed general steps: " << _pPlaner->_mLFootTrajectory.cols() << std::endl;

    int N = _mReference.cols();

    float T = 1.0f / (_pPlaner->getSamplesPerSecond());
	float g = 9.81;
    float hCoM = _pPlaner->getCoMHeight(); //0.7 as standard

	Eigen::Matrix3f A;
	Eigen::MatrixXf B;
	Eigen::MatrixXf C;
	
	A.setIdentity();
// *  this should be the correct version A={{1,0,0} | {T,1,0} | {T²/2,T,1}}
    A.col(1).x() = T;
    A.col(2).x() = T*T/2;
    A.col(2).y() = T;

    // this one worked previously
/*    A.col(0).x() = T;
    A.col(1).y() = T;
    A.col(2).x() = T*T/2;
*/

    // B = {T³/6, T²/2, T}
	B.resize(3,1);
	B << pow(T,3)/6, pow(T,2)/2, T;

    // C = {{1} | {0} | {-hCoM/g}}
	C.resize(1,3);
	C << 1, 0, -1*hCoM/g;

    Eigen::MatrixXf PX;
    Eigen::MatrixXf PU;
    Eigen::MatrixXf PUt;

    PX.resize(N,3);
    PU.resize(N,N);
    PX.setZero();
    PU.setZero();

    Eigen::Matrix3f tempM;
    std::vector<Eigen::Matrix3f> powerOfA;
    tempM.setIdentity();
    // TEST
    // tempM = A;

    // build PX:=[CA; CA^2; CA^3; ... CA^N]
    for (int i=0; i<N; i++)
    {
        powerOfA.push_back(tempM);  // save A^i for later use
        tempM = tempM*A;            // tempM = A^(i)*A = A^(i+1)
        PX.block(i,0,1,3) = C*tempM;
    }
    powerOfA.push_back(tempM);

    Eigen::Matrix3f dummy;
    Eigen::MatrixXf res;
    //float dummyFloat = 0;
    for (int i=0; i<N; i++)
    {
        for (int j=0; j<i; j++) //TODO: Endbedingung überprüfen
        {
            dummy = powerOfA[i-j];
            res=((C*dummy)*B);
            PU(i,j) = res(0,0);
        }
    }

    PUt = PU.transpose();

    float Q,R;
    Q = 1.0f;
    R = 1e-6;

    Eigen::MatrixXf w, wi, vX, vY, id, initX, initY, zmpRefX, zmpRefY, jerkX, jerkY;
    /*w.resize(N,N);
    v.resize(N,1);*/
    id.resize(N,N);
    id.setIdentity();
    // load reference ZMP values
    zmpRefX = _mReference.row(0).transpose();
    zmpRefY = _mReference.row(1).transpose();
    // set initial values for CoM, where every variable a has 3 dim in form of (a, a', a'')
    initX.resize(3,1);
    initX.setZero();
    initY.resize(3,1);
    initY.setZero();
    initX(0) = zmpRefX(0);
    initY(0) = zmpRefY(0);

    /* hack
    initX(1) = 0.001;
    initY(1) = 0.001;*/

    std::cout << "Using initial values of [" << initX.col(0).x() << "|" << initY.col(0).x() << "] for CoM!" << std::endl;
    //initX(0,0,1,1) = zmpRefX(0,0,1,1);
    //initY(0,0,1,1) = zmpRefY(0,0,1,1);


    w = Q * PUt * PU + R*id;
    vX = Q * PUt * PX * initX - Q*PUt * zmpRefX;
    vY = Q * PUt * PX * initY - Q*PUt * zmpRefY;

    wi = w.inverse();

    std::cout << "Size of w: [" << w.rows() << "|" << w.cols()  << "]" << std::endl;
    std::cout << "Size of vX: [" << vX.rows() << "|" << vX.cols()  << "]" << std::endl;

    jerkX = - wi * vX;
    jerkY = - wi * vY;

    // now compute CoM and realZMP
    Eigen::MatrixXf CX, CY, ZX, ZY, cxk, cyk; //, zxk, zyk;
    float zxk, zyk;
    CX.resize(N,3); // CoM X, X', X''
    CY.resize(N,3); // CoM Y, Y', Y''
    ZX.resize(N,1); // ZMP X
    ZY.resize(N,1); // ZMP Y

    cxk.resize(3,1);
    cyk.resize(3,1);
    //zxk.resize(1,1);
    //zyk.resize(1,1);

    cxk.setZero();
    cyk.setZero();
    cxk = initX;    // initial CoM values x, x', x''
    cyk = initY;    // initial CoM values y, y', y''

    // 0.085698, -0.000523125, 0.811306
    /* Unfortunately this is not working in debug mode, therefore we implement a hack
    float dummyX = _pPlaner->getInitialCoMPosition().x() / 1000.0f;
    float dummyY = _pPlaner->getInitialCoMPosition().y() / 1000.0f;
    float dummyZ = _pPlaner->getInitialCoMPosition().z() / 1000.0f;
    cxk(0) = dummyX;
    cyk(1) = dummyY;
    std::cout << "Initial CoM Position of Robot: [" << dummyX << ", " << dummyY << ", " << dummyZ << "]" << std::endl;
    */

    /*cxk(0) = zmpRefX(0,0);
    cyk(0) = zmpRefY(0,0);*/


    for (int i=0; i<N; i++)
    {
        cxk=A*cxk+B*jerkX(i);
        cyk=A*cyk+B*jerkY(i);

        zxk = (C*cxk).col(0).x();
        zyk = (C*cyk).col(0).x();

        CX.block(i,0,1,3) = cxk.transpose();
        CY.block(i,0,1,3) = cyk.transpose();

        ZX(i) = zxk;
        ZY(i) = zyk;
        //ZX(i,0) = zxk(0,0);
        //ZY(i,0) = zyk(0,0);
    }
    _mZMP.resize(2,N);
    _mZMP.block(0,0,1,N) = ZX.transpose();
    _mZMP.block(1,0,1,N) = ZY.transpose();

    Eigen::MatrixXf height;
    height.resize(1,N);
    height.setConstant(hCoM);

    _mCoM.resize(3,N);
    _mCoM.block(0,0,1,N) = CX.block(0,0,N,1).transpose();
    _mCoM.block(1,0,1,N) = CY.block(0,0,N,1).transpose();

    //_mCoM *= -1000;
    _mCoM.block(2,0,1,N) = height;

    std::cout << std::endl << "\tZMPPreviewControl::computeCom(): _mCoM(0): ["
              << _mCoM(0,0) << "," << _mCoM(1,0) << "," << _mCoM(2,0) << "], _mZMP(0): ["
              << _mZMP(0,0) << "," << _mZMP(1,0) << "," << "], given height ["
              << hCoM << "]" << std::endl << std::endl;

    std::cout << std::endl << "\tZMPPreviewControl::computeCom(): _mCoM(200): ["
              << _mCoM(0,200) << "," << _mCoM(1,200) << "," << _mCoM(2,200) << "], _mZMP(200): ["
              << _mZMP(0,200) << "," << _mZMP(1,200) << "," << "], given height ["
              << hCoM << "]" << std::endl << std::endl;

    std::cout << std::endl << "\tZMPPreviewControl::computeCom(): _mCoM(N-1): ["
              << _mCoM(0,N-1) << "," << _mCoM(1,N-1) << "," << _mCoM(2,N-1) << "], _mZMP(N-1): ["
              << _mZMP(0,N-1) << "," << _mZMP(1, N-1) << "," << "], given height ["
              << hCoM << "]" << std::endl << std::endl;

    buildRealZMPVisualization();
    buildCoMVisualization();



    //computeRealZMP(); // this is already done
}


void ZMPPreviewControl::computeRealZMP() {

}
