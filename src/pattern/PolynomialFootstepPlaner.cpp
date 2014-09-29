#include "pattern/PolynomialFootstepPlaner.h"

#include <VirtualRobot/Robot.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>

namespace Bipedal
{

PolynomialFootstepPlaner::PolynomialFootstepPlaner(const VirtualRobot::RobotNodePtr& leftFootBody,
                                                   const VirtualRobot::RobotNodePtr& rightFootBody)
    : FootstepPlaner(leftFootBody, rightFootBody)
{
}

void PolynomialFootstepPlaner::computeStep(double ssTime,
                                           double sampleDelta,
                                           double stepLength,
                                           double stepHeight,
                                           Eigen::Matrix6Xf& trajectory)
{
    // temporary variables
    double T1 = ssTime;                 // T^1
    double T2 = T1 * T1;                // T^2
    double T3 = T2 * T1;                // T^3
    double T4 = T3 * T1;                // T^4
    double S = stepLength;              // StepLength
    double H = stepHeight;              // Height
    double cx = -2 * S / T3;
    double dx = 3 * S / T2;
    double bz = 16 * H / T4;
    double cz = -32 * H / T3;
    double dz = 16 * H / T2;
    double ytemp, ztemp, x1, x2, x3, x4;

    // calculating foot trajectory for swinging leg including resting phase
    for (int i = 0; i < trajectory.cols(); i++)
    {
        x1 = sampleDelta * i;
        x2 = x1 * x1;
        x3 = x2 * x1;
        x4 = x3 * x1;

        if (x1 < ssTime)
        {
            // swinging phase
            ytemp =           cx * x3 + dx * x2;
            ztemp = bz * x4 + cz * x3 + dz * x2;
        }
        else
        {
            // resting phase continues with last values
            ytemp = S;
            ztemp = 0;
        }

        trajectory(0, i) = 0;
        trajectory(1, i) = ytemp;
        trajectory(2, i) = ztemp;
    }
}

void PolynomialFootstepPlaner::computeGeneralizedTrajectories()
{
    // total number of samples for each phase
    int iSS             = (int) (_iSampleSize * _dSingleSupportPhase);
    int iDS             = (int) (_iSampleSize * _dDoubleSupportPhase);
    int iSamplesPerStep = iSS + iDS;
    double sampleDelta  = 1.0 / _iSampleSize;

    std::cout << "Calculating generalized foot positions (iDS/iSS: [" << iDS << "|" << iSS << "])..." << std::flush;

    _mFootTrajectory      = Eigen::Matrix6Xf::Zero(6, iSamplesPerStep);
    _mFootTrajectoryFirst = Eigen::Matrix6Xf::Zero(6, iSamplesPerStep);
    _mFootTrajectoryLast  = Eigen::Matrix6Xf::Zero(6, iSamplesPerStep);

    computeStep(_dSingleSupportPhase, sampleDelta, _dStepLength, _dStepHeight, _mFootTrajectory);

    // half the hight/step length
    _mFootTrajectoryLast  = _mFootTrajectory / 2.0;
    _mFootTrajectoryFirst = _mFootTrajectory / 2.0;

    std::cout << " ok." << std::endl;
}

void PolynomialFootstepPlaner::computeFeetTrajectories()
{
    BOOST_ASSERT(_iNumberOfSteps >= 2);

    int iSS             = (int) (_iSampleSize * _dSingleSupportPhase);
    int iDS             = (int) (_iSampleSize * _dDoubleSupportPhase);
    int iSamplesPerStep = iSS + iDS;
    // first/last dual support phase needs to be *long* to warm up the preview control
    int iLDS = _iSampleSize;
    int iSamples = iSamplesPerStep * _iNumberOfSteps + 2*iLDS;

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
    Eigen::Vector6f vLeftFoot  = Eigen::Vector6f::Zero();
    Eigen::Vector6f vRightFoot = Eigen::Vector6f::Zero();
    vLeftFoot.x()  = -_dStepWidth / 2;
    vRightFoot.x() = _dStepWidth / 2;

    // starting with full DS-Phase
    _supportIntervals.emplace_back(0, iLDS, Bipedal::SUPPORT_BOTH);
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
                                       bLeft ? Bipedal::SUPPORT_RIGHT : Bipedal::SUPPORT_LEFT);
        _supportIntervals.emplace_back(index + iSS,
                                       index + iSS + iDS,
                                       Bipedal::SUPPORT_BOTH);
        // Complex const initialization using a lambda function
        // C++11 fuck yeah.
        const auto& _currentFootTrajectory = [this](unsigned i, unsigned numberOfSteps)
        {
            if (i == 0)
            {
                return _mFootTrajectoryFirst;
            }
            else if (i < numberOfSteps - 1)
            {
                return _mFootTrajectory;
            }
            else
            {
                return _mFootTrajectoryLast;
            }
        }(i, _iNumberOfSteps);

        for (int j = 0; j < iSamplesPerStep; j++)
        {
            // Make sure that RPY in in the trajectory are always ZERO
            // because you can not add RPY angles
            const Eigen::Vector6f currentLFoot = bLeft ? vLeftFoot + _currentFootTrajectory.col(j) : vLeftFoot;
            const Eigen::Vector6f currentRFoot = bLeft ? vRightFoot                                : vRightFoot + _currentFootTrajectory.col(j);
            _mLFootTrajectory.col(index) = currentLFoot;
            _mRFootTrajectory.col(index) = currentRFoot;
            index++;
        }

        // switch feet
        bLeft = !bLeft;
        // save new foot positions to temporary vectors
        vLeftFoot = _mLFootTrajectory.col(index - 1);
        vRightFoot = _mRFootTrajectory.col(index - 1);
    }

    // insert ending DS-phase
    _supportIntervals.emplace_back(index, index + iLDS, Bipedal::SUPPORT_BOTH);
    Eigen::Vector6f lastLeftPose  = _mLFootTrajectory.col(iLDS + _iNumberOfSteps*iSamplesPerStep - 1);
    Eigen::Vector6f lastRightPose = _mRFootTrajectory.col(iLDS + _iNumberOfSteps*iSamplesPerStep - 1);
    for (int j = 0; j < iLDS; j++) {
        _mLFootTrajectory.col(index) = lastLeftPose;
        _mRFootTrajectory.col(index) = lastRightPose;
        index++;
    }

    // If we want to walk in a circle transform foot trajectories by rotating around
    // a circle.
    // only do all that copying if necessary
    if (std::abs(_dAngle) > std::numeric_limits<double>::epsilon())
    {
        const double radiusLeft  = _dAngle > 0 ? _dRadius : _dRadius + _dStepWidth;
        const double radiusRight = _dAngle > 0 ? _dRadius + _dStepWidth : _dRadius;
        const double circumfenceLeft = _dAngle * radiusLeft;
        const double circumfenceRight = _dAngle * radiusRight;
        const double circPerYLeft  = circumfenceLeft  / vLeftFoot.y();
        const double circPerYRight = circumfenceRight / vRightFoot.y();
        const Eigen::Vector3f center(_dAngle > 0 ? -_dRadius : _dRadius, 0, 0);

        // Maybe split loop in left/right for cache speedup
        Eigen::Matrix4f leftFootPose = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f rightFootPose = Eigen::Matrix4f::Identity();
        Eigen::Vector6f rotatedLeft;
        Eigen::Vector6f rotatedRight;
        for (int k = 0; k < index; k++)
        {
            Eigen::Matrix4f R = Eigen::Matrix4f::Identity();

            const Eigen::Vector6f& currentLFoot = _mLFootTrajectory.col(k);
            const Eigen::Vector6f& currentRFoot = _mRFootTrajectory.col(k);
            VirtualRobot::MathTools::rpy2eigen4f(currentLFoot[3], currentLFoot[4], currentLFoot[5], leftFootPose);
            VirtualRobot::MathTools::rpy2eigen4f(currentRFoot[3], currentRFoot[4], currentRFoot[5], rightFootPose);

            Eigen::Vector3f tmp;

            // transform left foot pose
            const double alphaLeft = currentLFoot.y() * circPerYLeft / radiusLeft;
            VirtualRobot::MathTools::rpy2eigen4f(0, 0, alphaLeft, R);
            leftFootPose *= R;
            rotatedLeft.x() = radiusLeft * cos(alphaLeft) + center.x();
            rotatedLeft.y() = radiusLeft * sin(alphaLeft) + center.y();
            rotatedLeft.z() = currentLFoot.z();
            VirtualRobot::MathTools::eigen4f2rpy(leftFootPose, tmp);
            rotatedLeft.tail(3) = tmp;

            // transform right foot pose
            const double alphaRight = currentRFoot.y() * circPerYRight / radiusRight;
            VirtualRobot::MathTools::rpy2eigen4f(0, 0, alphaRight, R);
            rightFootPose *= R;
            rotatedRight.x() = radiusRight * cos(alphaRight) + center.x();
            rotatedRight.y() = radiusRight * sin(alphaRight) + center.y();
            rotatedRight.z() = currentRFoot.z();
            VirtualRobot::MathTools::eigen4f2rpy(rightFootPose, tmp);
            rotatedRight.tail(3) = tmp;

            _mLFootTrajectory.col(k) = rotatedLeft;
            _mRFootTrajectory.col(k) = rotatedRight;
        }
    }
}

}

