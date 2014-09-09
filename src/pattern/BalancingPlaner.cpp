#include "pattern/BalancingPlaner.h"

#include <VirtualRobot/Robot.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>

#include "utils/Interpolation.h"

BalancingPlaner::BalancingPlaner(const VirtualRobot::RobotNodePtr& leftFootBody,
                                 const VirtualRobot::RobotNodePtr& rightFootBody)
    : FootstepPlaner(leftFootBody, rightFootBody)
{
}

void BalancingPlaner::computeStep(double ssTime,
                                  double sampleDelta,
                                  double stepLength,
                                  double stepHeight,
                                  double lateralDirection,
                                  Eigen::Matrix6Xf& trajectory)
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
        // precompute {c*x, d*x, b*z, c*z, d*z, x^1, x^2, x^3, x^4} for optimization purposes
        cx = -2 * S / T3;
        dx = 3 * S / T2;
        bz = 16 * H / T4;
        cz = -32 * H / T3;
        dz = 16 * H / T2;
        x1 = sampleDelta * i;
        x2 = x1 * x1;
        x3 = x2 * x1;
        x4 = x3 * x1;

        if (x1 < ssTime)
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

/**
 * Lift foot and move stepWidth/4 in the lateral direction.
 */
void BalancingPlaner::computeInitialStep(double sampleDelta,
                                         double stepWidth,
                                         double stepHeight,
                                         double lateralDirection,
                                         Eigen::Matrix6Xf& trajectory)
{
    unsigned numSamples = trajectory.cols();
    double T = sampleDelta * numSamples;
    for (unsigned i = 0; i < numSamples; i++)
    {
        double t = sampleDelta*i;
        trajectory(0, i) = -lateralDirection * Bipedal::splinePositionInterpolation(t, T, stepWidth/4, 0, 0);
        trajectory(1, i) = 0;
        trajectory(2, i) = Bipedal::splinePositionInterpolation(t, T, stepHeight, 0, 0);
    }
}

void BalancingPlaner::computeLastStep(double sampleDelta,
                                      double stepWidth,
                                      double stepHeight,
                                      double lateralDirection,
                                      Eigen::Matrix6Xf& trajectory)
{
    unsigned numSamples = trajectory.cols();
    double T = sampleDelta * numSamples;
    for (unsigned i = 0; i < numSamples; i++)
    {
        double t = sampleDelta*i;
        trajectory(0, i) = lateralDirection * Bipedal::splinePositionInterpolation(t, T, stepWidth/4, 0, 0);
        trajectory(1, i) = 0;
        trajectory(2, i) = -Bipedal::splinePositionInterpolation(t, T, stepHeight, 0, 0);
    }
}

void BalancingPlaner::computeGeneralizedTrajectories()
{
    // total number of samples for each phase
    int iSS             = (int) (_iSampleSize * _dSingleSupportPhase);
    int iDS             = (int) (_iSampleSize * _dDoubleSupportPhase);
    int iSamplesPerStep = iSS + iDS;
    double sampleDelta  = 1.0 / _iSampleSize;

    std::cout << "Calculating generalized foot positions (iDS/iSS: [" << iDS << "|" << iSS << "])..." << std::flush;

    _mFootTrajectoryLeft  = Eigen::Matrix6Xf::Zero(6, iSamplesPerStep);
    _mFootTrajectoryRight = Eigen::Matrix6Xf::Zero(6, iSamplesPerStep);
    _mFootTrajectoryFirst = Eigen::Matrix6Xf::Zero(6, iSamplesPerStep);
    _mFootTrajectoryLast  = Eigen::Matrix6Xf::Zero(6, iSamplesPerStep);


    bool bRightFootLast = _iNumberOfSteps % 2 != 0;
    computeStep(_dSingleSupportPhase, sampleDelta, _dStepLength, _dStepHeight, 1.0, _mFootTrajectoryLeft);
    computeStep(_dSingleSupportPhase, sampleDelta, _dStepLength, _dStepHeight, -1.0, _mFootTrajectoryRight);
    computeInitialStep(sampleDelta, _dStepWidth, _dStepHeight, _bLeftFootFirst ? -1.0 : 1.0, _mFootTrajectoryFirst);
    computeLastStep(sampleDelta, _dStepWidth, _dStepHeight, bRightFootLast ? 1.0 : -1.0, _mFootTrajectoryLast);

    std::cout << " ok." << std::endl;
}

// FIXME This only works or exactly 2 steps.
void BalancingPlaner::computeFeetTrajectories()
{
    BOOST_ASSERT(_iNumberOfSteps >= 2);

    int iSS             = (int) (_iSampleSize * _dSingleSupportPhase);
    int iDS             = (int) (_iSampleSize * _dDoubleSupportPhase);
    int iSamplesPerStep = iSS + iDS;
    // first/last dual support phase needs to be *long* to warm up the preview control
    int iLDS = _iSampleSize;
    int iSamples = iSamplesPerStep * _iNumberOfSteps + 2*iLDS + 5*iLDS;

    std::cout << "Calculating " << _iNumberOfSteps << " using " << iSamplesPerStep << " steps." << std::endl;
    std::cout << "Using " << iLDS << " samples for the start and end DS phase." << std::endl;

    // FIXME We could actually retain them if only the number of steps changed
    computeGeneralizedTrajectories();

    // ******************************************
    // ** calculate Foot Positions for n-Steps **
    // ******************************************

    // Each step has a SS + DS phase and we start with a DS and end with a DS
    _mLFootTrajectory =  Eigen::Matrix6Xf::Zero(6, iSamples);
    _mRFootTrajectory =  Eigen::Matrix6Xf::Zero(6, iSamples);
    _supportIntervals.clear();

    // determine starting positions of left and right foot
    Eigen::Vector6f vLeftFoot = Eigen::Vector6f::Zero();
    Eigen::Vector6f vRightFoot = Eigen::Vector6f::Zero();
    vLeftFoot.x()  = -_dStepWidth / 2;
    vRightFoot.x() = _dStepWidth / 2;

    // starting with full DS-Phase
    _supportIntervals.emplace_back(0, iLDS, Kinematics::SUPPORT_BOTH);
    int index = 0;
    for (int j = 0; j < iLDS; j++)
    {
        _mLFootTrajectory.col(index) = vLeftFoot;
        _mRFootTrajectory.col(index) = vRightFoot;
        index++;
    }

    // Actual walking phase
    bool bLeft = (_bLeftFootFirst ? true : false);
    for (int i = 0; i < _iNumberOfSteps; i++)
    {
        _supportIntervals.emplace_back(index,
                                       index + iSS,
                                       bLeft ? Kinematics::SUPPORT_RIGHT : Kinematics::SUPPORT_LEFT);
        if (i == 0 )
        {
            _supportIntervals.emplace_back(index + iSS,
                                           index + iSS + iDS,
                                           bLeft ? Kinematics::SUPPORT_RIGHT : Kinematics::SUPPORT_LEFT);
        }
        else
        {
            _supportIntervals.emplace_back(index + iSS,
                                           index + iSS + iDS,
                                           Kinematics::SUPPORT_BOTH);
        }
        // Complex const initialization using a lambda function
        // C++11 fuck yeah.
        const Eigen::Matrix6Xf& _currentFootTrajectory = [this](unsigned i, unsigned numberOfSteps, bool left)
        {
            if (i == 0)
            {
                return _mFootTrajectoryFirst;
            }
            else if (i < numberOfSteps - 1)
            {
                if (left)
                    return _mFootTrajectoryLeft;
                else
                    return _mFootTrajectoryRight;
            }
            else
            {
                return _mFootTrajectoryLast;
            }
        }(i, _iNumberOfSteps, bLeft);

        for (int j = 0; j < iSamplesPerStep; j++)
        {
            const Eigen::Vector6f currentLFoot = bLeft ? vLeftFoot + _currentFootTrajectory.col(j) : vLeftFoot;
            const Eigen::Vector6f currentRFoot = bLeft ? vRightFoot                                : vRightFoot + _currentFootTrajectory.col(j);
            _mLFootTrajectory.col(index) = currentLFoot;
            _mRFootTrajectory.col(index) = currentRFoot;
            index++;
        }

        // save new foot positions to temporary vectors
        vLeftFoot = _mLFootTrajectory.col(index - 1);
        vRightFoot = _mRFootTrajectory.col(index - 1);

        // FIXME Hack to remain on left foot
        if (i == 0)
        {
            _supportIntervals.emplace_back(index,
                                           index + iLDS*5,
                                           bLeft ? Kinematics::SUPPORT_RIGHT : Kinematics::SUPPORT_LEFT);
            for (int j = 0; j < iLDS*5; j++)
            {
                _mLFootTrajectory.col(index) = vLeftFoot;
                _mRFootTrajectory.col(index) = vRightFoot;
                index++;
            }
        }

        // switch feet
        // Don't switch after lifting left leg (step 1)
        // or before bringing down the lifted foot (last step)
        if (i != 0 || i != _iNumberOfSteps-2)
            bLeft = !bLeft;
    }

    // insert ending DS-phase
    _supportIntervals.emplace_back(index, index + iLDS, Kinematics::SUPPORT_BOTH);
    Eigen::Vector6f lastLeftPos  = _mLFootTrajectory.col(index - 1);
    Eigen::Vector6f lastRightPos = _mRFootTrajectory.col(index - 1);
    for (int j = 0; j < iLDS; j++) {
        _mLFootTrajectory.col(index) = lastLeftPos;
        _mRFootTrajectory.col(index) = lastRightPos;
        index++;
    }
}

