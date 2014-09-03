#include "pattern/BalancingPlaner.h"

#include <VirtualRobot/Robot.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>

#include "utils/Interpolation.h"

BalancingPlaner::BalancingPlaner()
    : PolynomialFootstepPlaner()
{
}

void BalancingPlaner::calculateStep(double ssTime,
                                    int ssSamples,
                                    double sampleDelta,
                                    double stepLength,
                                    double stepHeight,
                                    double lateralDirection,
                                    Eigen::Matrix3Xf& trajectory)
{
    // temporary variables
    double T1 = ssTime;                 // T^1
    double T2 = T1 * T1;                // T^2
    double T3 = T2 * T1;                // T^3
    double T4 = T3 * T1;                // T^4
    double S = stepLength;              // StepLength
    double H = stepHeight;              // Height
    double cx, dx, bz, cz, dz, xtemp, ztemp, ytemp, x1, x2, x3, x4;

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
            ytemp = lateralDirection * bz * x4 + cz * x3 + dz * x2;
        }
        else
        {
            // resting phase continues with last values
            xtemp = S;
            ytemp = 0;
            ztemp = 0;
        }

        trajectory(0, i) = 0;
        trajectory(1, i) = xtemp;
        trajectory(2, i) = ztemp;
    }
}

void BalancingPlaner::calculateInitialStep(double sampleDelta,
                                           double stepWidth,
                                           double stepHeight,
                                           double lateralDirection,
                                           Eigen::Matrix3Xf& trajectory)
{
    unsigned numSamples = trajectory.cols();
    double T = sampleDelta * numSamples;
    for (unsigned i = 0; i < numSamples; i++)
    {
        double t = sampleDelta*i;
        trajectory(0, i) = 0;
        trajectory(1, i) = lateralDirection * (stepWidth/2 - Bipedal::splinePositionInterpolation(t, T, stepWidth/2, 0, 0));
        trajectory(2, i) = Bipedal::splinePositionInterpolation(t, T, stepHeight, 0, 0);
    }
}

void BalancingPlaner::calculateLastStep(double sampleDelta,
                                        double stepWidth,
                                        double stepHeight,
                                        double lateralDirection,
                                        Eigen::Matrix3Xf& trajectory)
{
    unsigned numSamples = trajectory.cols();
    double T = sampleDelta * numSamples;
    for (unsigned i = 0; i < numSamples; i++)
    {
        double t = sampleDelta*i;
        trajectory(0, i) = 0;
        trajectory(1, i) = lateralDirection * Bipedal::splinePositionInterpolation(t, T, stepWidth/2, 0, 0);
        trajectory(2, i) = stepHeight - Bipedal::splinePositionInterpolation(t, T, stepHeight, 0, 0);
    }
}

void BalancingPlaner::computeFeetTrajectories(int numberOfSteps)
{
    /* Currently only going to standing on one foot and going back is supported
    if (numberOfSteps < 2)
    {
        numberOfSteps = 2;
    }
    */
    numberOfSteps = 2;

    _iNumberOfSteps = numberOfSteps;
    _mLFootPositions = Eigen::Matrix3Xf::Zero(3, _iNumberOfSteps + 2);
    _mRFootPositions = Eigen::Matrix3Xf::Zero(3, _iNumberOfSteps + 2);
    int stepCounter = 0;
    // ***************************************************
    // ** calculate generalized swinging leg trajectory **
    // ***************************************************
    int iSamplesPerStep = (int)(_iSampleSize * _dStepPeriod);
    double sampleDelta = 1.0f / _iSampleSize;
    // initialise generalized trajectory
    Eigen::Matrix3Xf _footTrajectoryLeft = Eigen::Matrix3Xf::Zero(3, iSamplesPerStep);
    Eigen::Matrix3Xf _footTrajectoryRight = Eigen::Matrix3Xf::Zero(3, iSamplesPerStep);
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
    int iFDS = iSS;

    bool _bRightFootLast = numberOfSteps % 2 == 0;
    calculateStep(_dSingleSupportPhase, iSS, sampleDelta, _dStepLength, _dStepHeight, 1.0, _footTrajectoryLeft);
    calculateStep(_dSingleSupportPhase, iSS, sampleDelta, _dStepLength, _dStepHeight, -1.0, _footTrajectoryRight);
    calculateInitialStep(sampleDelta, _dStepWidth, _dStepHeight, _bLeftFootFirst ? -1.0 : 1.0, _footTrajectoryFirst);
    calculateLastStep(sampleDelta, _dStepWidth, _dStepHeight, _bRightFootLast ? 1.0 : -1.0, _footTrajectoryLast);

    // ******************************************
    // ** calculate Foot Positions for n-Steps **
    // ******************************************
    // initialise Matrices
    std::cout << "calculate Foot Positions for " << _iNumberOfSteps << " Steps: (iDS/iSS: [" << iDS << "|" << iSS << "])" << std::endl;
    int iSamples = iSamplesPerStep * _iNumberOfSteps + 2*iFDS;
    std::cout << "generating a total of " << iSamples << " samples. Using " << iSamplesPerStep << " samples per step and [" << iSS << "|" << iDS << "] samples for [SS|DS]-phase!" << std::endl;
    _mLFootTrajectory =  Eigen::Matrix3Xf::Zero(3, iSamples);
    _mRFootTrajectory =  Eigen::Matrix3Xf::Zero(3, iSamples);
    bool bLeft = (_bLeftFootFirst ? true : false);
    int index = 0;
    // determine starting positions of left and right foot
    Eigen::Vector3f vLeftFoot;
    Eigen::Vector3f vRightFoot;
    Eigen::Vector3f vTempL, vTempR;
    vLeftFoot.setZero();
    vRightFoot.setZero();
    vLeftFoot(0) = -_dStepWidth / 2;
    vRightFoot(0) = _dStepWidth / 2;
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

    for (int i = 0; i < _iNumberOfSteps; i++)
    {
        // Complex const initialization using a lambda function
        // C++11 fuck yeah.
        const Eigen::Matrix3Xf& _currentFootTrajectory = [i, bLeft, numberOfSteps,
                                                          &_footTrajectoryFirst,
                                                          &_footTrajectoryLast,
                                                          &_footTrajectoryLeft,
                                                          &_footTrajectoryRight]()
        {
            if (i == 0)
            {
                return _footTrajectoryFirst;
            }
            else if (i < numberOfSteps - 1)
            {
                if (bLeft)
                {
                    return _footTrajectoryLeft;
                }
                else
                {
                    return _footTrajectoryRight;
                }
            }
            else
            {
                return _footTrajectoryLast;
            }
        }();

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

            Eigen::Vector3f currentLFoot = vLeftFoot + vTempL;
            Eigen::Vector3f currentRFoot = vRightFoot + vTempR;
            _mLFootTrajectory.col(index) = currentLFoot;
            _mRFootTrajectory.col(index) = currentRFoot;
            index++;
        }

        // switch feet
        bLeft = (bLeft ? false : true);
        // save new foot positions to temporary vectors
        vLeftFoot = _mLFootTrajectory.col(index - 1);
        vRightFoot = _mRFootTrajectory.col(index - 1);
        stepCounter++;
        _mLFootPositions.col(stepCounter) = _mLFootTrajectory.col(iFDS + (i+1)*iSamplesPerStep - 1);
        _mRFootPositions.col(stepCounter) = _mRFootTrajectory.col(iFDS + (i+1)*iSamplesPerStep - 1);
    }

    Eigen::Vector3f lastLeftPos = _mLFootTrajectory.col(iFDS + _iNumberOfSteps*iSamplesPerStep - 1);
    Eigen::Vector3f lastRighPos = _mRFootTrajectory.col(iFDS + _iNumberOfSteps*iSamplesPerStep - 1);

    // insert ending DS-phase
    for (int j=0; j<iFDS; j++) {
        _mLFootTrajectory.col(index) = lastLeftPos;
        _mRFootTrajectory.col(index) = lastRighPos;
        index++;
    }
    // save last step positions
    stepCounter++;
    _mLFootPositions.col(stepCounter) = lastLeftPos;
    _mRFootPositions.col(stepCounter) = lastRighPos;

    _bGenerated = true;
}


