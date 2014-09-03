#include "pattern/FootstepPlaner.h"

#include "utils/Walking.h"

#include <VirtualRobot/Robot.h>

#include <boost/assert.hpp>


FootstepPlaner::FootstepPlaner(void)
    : _dStepLength(0.3f)
    , _dStepWidth(0.2f)
    , _dStepHeight(0.1f)
    , _dStepPeriod(0.8f)
    , _dSingleSupportPhase(0.7f)
    , _dDoubleSupportPhase(0.1f)
    , _iSampleSize(50)
    , _bLeftFootFirst(true)
    , _bGenerated(false)
{
}

void FootstepPlaner::setParameters(unsigned numberOfSteps,
                                   unsigned sampleSize,
                                   double stepLength,
                                   double singleSupportPhase,
                                   double doubleSupportPhase,
                                   double stepHeight,
                                   double angle,
                                   double radius)
{
    _iNumberOfSteps = numberOfSteps;
    _dStepLength = stepLength;
    _dStepPeriod = singleSupportPhase + doubleSupportPhase;
    _dDoubleSupportPhase = doubleSupportPhase;
    _dSingleSupportPhase = singleSupportPhase;
    _dStepHeight = stepHeight;
    _iSampleSize = sampleSize;
    _bParametersInitialized = true;
    _bGenerated = false;
}

void FootstepPlaner::generate()
{
    BOOST_ASSERT(_bParametersInitialized);

    computeFeetTrajectories();
    transformFootPositions();

    _bGenerated = true;
}

// assign a Robot Model to the Footstep Planer
void FootstepPlaner::setRobotModel(const VirtualRobot::RobotPtr& pRobot,
                                   const std::string& nameRightFoot,
                                   const std::string& nameLeftFoot)
{
    _pRobot = pRobot;
    _sRightFootName = nameRightFoot;
    _sLeftFootName = nameLeftFoot;

    // compute shape of feet (support polygon of left and right foot)
    computeFeetShape();
}

// compute the shape of each foot and generate a visualization for every foot
void FootstepPlaner::computeFeetShape()
{
    Eigen::Vector2f vRightCenter;
    Eigen::Vector2f vLeftCenter;
    cvRight = Walking::ComputeFootContact(_pRobot->getRobotNode(_sRightFootName)->getCollisionModel());
    cvLeft  = Walking::ComputeFootContact(_pRobot->getRobotNode(_sLeftFootName)->getCollisionModel());
    vRightCenter = Walking::CenterConvexHull(cvRight);
    vLeftCenter  = Walking::CenterConvexHull(cvLeft);
    _vRightFootCenter = vRightCenter / 1000.0f;
    _vLeftFootCenter  = vLeftCenter / 1000.0f;
    _mRotateWalking = Walking::ComputeWalkingDirection(vLeftCenter, vRightCenter);
}

// build visualization for feet position and feet trajectories
void FootstepPlaner::transformFootPositions()
{
    BOOST_ASSERT(_bGenerated);

    // rotate generated walking pattern to walking direction
    Eigen::Matrix3f rot;
    rot.setIdentity();
    rot.block(0, 0, 2, 2) = _mRotateWalking;
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
    for (int i = 0; i < rFootPositions.cols(); i++)
    {
        rFootPositions.col(i) += rDelta ;
    }

    for (int i = 0; i < lFootPositions.cols(); i++)
    {
        lFootPositions.col(i) += lDelta ;
    }

    for (int i = 0; i < rFootTrajectories.cols(); i++)
    {
        rFootTrajectories.col(i) += rDelta ;
    }

    for (int i = 0; i < lFootTrajectories.cols(); i++)
    {
        lFootTrajectories.col(i) += lDelta ;
    }

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

const Eigen::Matrix3Xf& FootstepPlaner::getLeftFootPositions() const
{
    BOOST_ASSERT(_bGenerated);
    return _mLFootPositionsTransformed;
}

const Eigen::Matrix3Xf& FootstepPlaner::getRightFootPositions() const
{
    BOOST_ASSERT(_bGenerated);
    return _mRFootPositionsTransformed;
}


const Eigen::Matrix3Xf& FootstepPlaner::getLeftFootTrajectory() const
{
    BOOST_ASSERT(_bGenerated);
    return _mLFootTrajectoryTransformed;
}

const Eigen::Matrix3Xf& FootstepPlaner::getRightFootTrajectory() const
{
    BOOST_ASSERT(_bGenerated);
    return _mRFootTrajectoryTransformed;
}

const std::vector<Kinematics::SupportInterval>& FootstepPlaner::getSupportIntervals() const
{
    BOOST_ASSERT(_bGenerated);
    return _supportIntervals;
}

