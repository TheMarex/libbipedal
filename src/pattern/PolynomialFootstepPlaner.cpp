#include "pattern/PolynomialFootstepPlaner.h"

#include <VirtualRobot/Robot.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>

PolynomialFootstepPlaner::PolynomialFootstepPlaner()
    : FootstepPlaner()
{
}

void PolynomialFootstepPlaner::setParameters(double stepLength, double stepPeriod, double doubleSupportPhase, double stepHeight, int sampleSize)
{
    if (_bParametersInitialized)
    {
        //TODO: first delete all array, vectors and matrices
    }

    _dStepLength = stepLength;
    _dStepPeriod = stepPeriod;
    _dDoubleSupportPhase = doubleSupportPhase;
    _dSingleSupportPhase = _dStepPeriod - _dDoubleSupportPhase;
    _dStepHeight = stepHeight;
    _iSampleSize = sampleSize;
    _bParametersInitialized = true;
}

// start walking with left foot first
void PolynomialFootstepPlaner::setLeftFootFirst()
{
    _bLeftFootFirst = true;
}

// start walking with right foot first
void PolynomialFootstepPlaner::setRightFootFirst()
{
    _bLeftFootFirst = false;
}

void PolynomialFootstepPlaner::calculateStep(double ssTime, int ssSamples, double sampleDelta, double stepLength, double stepHeight, Eigen::Matrix3Xf& trajectory)
{
    // temporary variables
    double T1 = ssTime;                 // T^1
    double T2 = T1 * T1;                // T^2
    double T3 = T2 * T1;                // T^3
    double T4 = T3 * T1;                // T^4
    double S = stepLength;              // StepLength
    double H = stepHeight;              // Height
    double cx, dx, bz, cz, dz, xtemp, ztemp, x1, x2, x3, x4;

    // calculating foot trajectory for swinging leg including resting phase
    for (int i = 0; i < trajectory.cols(); i++)
    {
        // precalculate {c*x, d*x, b*z, c*z, d*z, x^1, x^2, x^3, x^4} for optimization purposes
        cx = -2 * S / T3;
        dx = 3 * S / T2;
        bz = 16 * H / T4;
        cz = -32 * H / T3;
        dz = 16 * H / T2;
        x1 = sampleDelta * i;
        x2 = x1 * x1;
        x3 = x2 * x1;
        x4 = x3 * x1;

        if (i < ssSamples)
        {
            // swinging phase
            xtemp =       cx * x3 + dx * x2;
            ztemp = bz * x4 + cz * x3 + dz * x2;
        }
        else
        {
            // resting phase continues with last values
            xtemp = S;
            ztemp = 0;
        }

        trajectory(0, i) = xtemp;
        trajectory(1, i) = 0;
        trajectory(2, i) = ztemp;
    }
}

void PolynomialFootstepPlaner::computeFeetTrajectories(int numberOfSteps)
{
    if (numberOfSteps < 2)
    {
        numberOfSteps = 2;
    }

    _iNumberOfSteps = numberOfSteps;
    _mLFootPositions = Eigen::Matrix3Xf::Zero(3, _iNumberOfSteps + 1);
    _mRFootPositions = Eigen::Matrix3Xf::Zero(3, _iNumberOfSteps + 1);
    int stepCounter = 0;
    // ***************************************************
    // ** calculate generalized swinging leg trajectory **
    // ***************************************************
    int iSamplesPerStep = (int)(_iSampleSize * _dStepPeriod);
    double sampleDelta = 1.0f / _iSampleSize;
    // initialise generalized trajectory
    Eigen::Matrix3Xf _footTrajectory = Eigen::Matrix3Xf::Zero(3, iSamplesPerStep);
    // generalized trajectory for the first and last swinging leg
    Eigen::Matrix3Xf _footTrajectoryFirst = Eigen::Matrix3Xf::Zero(3, iSamplesPerStep);
    Eigen::Matrix3Xf _footTrajectoryLast = Eigen::Matrix3Xf::Zero(3, iSamplesPerStep);
    // percentage of different Phases
    double _dSS = (double)(_dSingleSupportPhase / _dStepPeriod);
    double _dDS = (double)(_dDoubleSupportPhase / _dStepPeriod);
    // total number of samples for each phase
    int iSS = (int)(iSamplesPerStep * _dSS);
    int iDS = (int)(iSamplesPerStep * _dDS);
    // first dual support phase needs to be *long* to warm up the preview control
    int iFDS = iDS * 50;

    calculateStep(_dSingleSupportPhase, iSS, sampleDelta, _dStepLength, _dStepHeight, _footTrajectory);

    // half the hight/step length
    _footTrajectoryLast = _footTrajectory / 2.0;

    // first step has only 3/4 of the lift-off time
    double firstStepFactor = 3 / 4.0;
    double firstStepSSPhase = _dSingleSupportPhase * firstStepFactor;
    int iFSS = iSS * firstStepFactor;
    int iWait = iSS - iFSS;
    int iFirstStepSampels = iSamplesPerStep - iWait;
    Eigen::Matrix3Xf _tempTraj = Eigen::Matrix3Xf::Zero(3, iFirstStepSampels);
    calculateStep(firstStepSSPhase, iFSS, sampleDelta, _dStepLength / 2.0, _dStepHeight / 2.0, _tempTraj);
    _footTrajectoryFirst.block(0, iSamplesPerStep - iFirstStepSampels, 3, _tempTraj.cols()) = _tempTraj;

    for (int i = 0; i < iWait; i++)
    {
        _footTrajectoryFirst(0, i) = _tempTraj(0, 0);
        _footTrajectoryFirst(2, i) = _tempTraj(2, 0);
    }

    // ******************************************
    // ** calculate Foot Positions for n-Steps **
    // ******************************************
    // initialise Matrices
    std::cout << "calculate Foot Positions for " << _iNumberOfSteps << " Steps: (iDS/iSS: [" << iDS << "|" << iSS << "])" << std::endl;
    int iSamples = iSamplesPerStep * _iNumberOfSteps + iFDS;
    std::cout << "generating a total of " << iSamples << " samples. Using " << iSamplesPerStep << " samples per step and [" << iSS << "|" << iDS << "] samples for [SS|DS]-phase!" << std::endl;
    _mLFootTrajectory =  Eigen::Matrix3Xf::Zero(3, iSamples);
    _mRFootTrajectory =  Eigen::Matrix3Xf::Zero(3, iSamples);
    bool bLeft = (_bLeftFootFirst ? true : false);
    bool halfStep = true;
    int index = 0;
    // determine starting positions of left and right foot
    Eigen::Vector3f vLeftFoot;
    Eigen::Vector3f vRightFoot;
    Eigen::Vector3f vTempL, vTempR;
    vLeftFoot.setZero();
    vRightFoot.setZero();
    vLeftFoot(1) = -_dStepWidth / 2;
    vRightFoot(1) = _dStepWidth / 2;
    _vDeltaLeftFoot = vLeftFoot;
    _vDeltaRightFoot = vRightFoot;

    _mLFootPositions.col(stepCounter) = vLeftFoot;
    _mRFootPositions.col(stepCounter) = vRightFoot;

    // starting with full DS-Phase
    for (int j = 0; j < iFDS; j++)
    {
        _mLFootTrajectory.col(index) = vLeftFoot;
        _mRFootTrajectory.col(index) = vRightFoot;
        index++;
    }

    Eigen::Matrix3Xf _currentFootTrajectory = _footTrajectoryFirst;

    for (int i = 0; i < _iNumberOfSteps; i++)
    {
        if (i == 1)
        {
            _currentFootTrajectory = _footTrajectory;
        }
        else if (i == _iNumberOfSteps - 1)
        {
            _currentFootTrajectory = _footTrajectoryLast;
        }

        // TODO: find a more elegant solution
        vTempL.setZero();
        vTempR.setZero();

        for (int j = 0; j < iSamplesPerStep; j++)
        {
            // which foot do we move?
            if (bLeft)
            {
                vTempL = _currentFootTrajectory.col(j);
            }
            else
            {
                vTempR = _currentFootTrajectory.col(j);
            }

            _mLFootTrajectory.col(index) = vLeftFoot + vTempL;
            _mRFootTrajectory.col(index) = vRightFoot + vTempR;
            index++;
        }

        // switch feet
        bLeft = (bLeft ? false : true);
        // save new foot positions to temporary vectors
        vLeftFoot = _mLFootTrajectory.col(index - 1);
        vRightFoot = _mRFootTrajectory.col(index - 1);
        stepCounter++;
        _mLFootPositions.col(stepCounter) = vLeftFoot;
        _mRFootPositions.col(stepCounter) = vRightFoot;
    }

    // insert ending DS-phase
    /* this is too much
    for (int j=0; j<iDS; j++) {
        _mLFootTrajectory.col(index) = vLeftFoot;
        _mRFootTrajectory.col(index) = vRightFoot;
        index++;
    }*/
    // save last step positions
    /*stepCounter++;
    _mLFootPositions.col(stepCounter)=vLeftFoot;
    _mRFootPositions.col(stepCounter)=vRightFoot;*/
    _bGenerated = true;
}


