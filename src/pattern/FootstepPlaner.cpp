#include "pattern/FootstepPlaner.h"

#include "utils/Walking.h"

#include <VirtualRobot/Robot.h>

#include <boost/assert.hpp>

namespace Bipedal
{

FootstepPlaner::FootstepPlaner(const VirtualRobot::RobotNodePtr& leftFootBody,
                               const VirtualRobot::RobotNodePtr& rightFootBody)
    : _dStepLength(0.3f)
    , _dStepWidth(0.2f)
    , _dStepHeight(0.1f)
    , _dStepPeriod(0.8f)
    , _dSingleSupportPhase(0.7f)
    , _dDoubleSupportPhase(0.1f)
    , _iSampleSize(50)
    , _bLeftFootFirst(true)
    , _bGenerated(false)
    , _pLeftFootBody(leftFootBody)
    , _pRightFootBody(rightFootBody)
{
    computeFeetShape();
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
    _dAngle = angle;
    _dRadius = radius;
    _bGenerated = false;
}

void FootstepPlaner::generate(const Eigen::Matrix4f& leftFootPose, const Eigen::Matrix4f& rightFootPose)
{
    BOOST_ASSERT(_bParametersInitialized);

    computeFeetTrajectories();
    transformFootTrajectories(leftFootPose, rightFootPose);

    _bGenerated = true;
}

// compute the shape of each foot and generate a visualization for every foot
void FootstepPlaner::computeFeetShape()
{
    Eigen::Vector2f vRightCenter;
    Eigen::Vector2f vLeftCenter;
    cvRight = Bipedal::ComputeFootContact(_pRightFootBody->getCollisionModel());
    cvLeft  = Bipedal::ComputeFootContact(_pLeftFootBody->getCollisionModel());
    vRightCenter = Bipedal::CenterConvexHull(cvRight);
    vLeftCenter  = Bipedal::CenterConvexHull(cvLeft);
    _vRightFootCenter = vRightCenter / 1000.0f;
    _vLeftFootCenter  = vLeftCenter / 1000.0f;
}

// build visualization for feet position and feet trajectories
void FootstepPlaner::transformFootTrajectories(const Eigen::Matrix4f& leftFootPose, const Eigen::Matrix4f& rightFootPose)
{
    Eigen::Matrix4f trajLeftFootPose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f trajRightFootPose = Eigen::Matrix4f::Identity();
    Eigen::Vector6f startLeft  = _mLFootTrajectory.col(0);
    Eigen::Vector6f startRight = _mRFootTrajectory.col(0);
    VirtualRobot::MathTools::rpy2eigen4f(startLeft[3], startLeft[4], startLeft[5], trajLeftFootPose);
    VirtualRobot::MathTools::rpy2eigen4f(startRight[3], startRight[4], startRight[5], trajRightFootPose);
    trajLeftFootPose.block(0, 3, 3, 1)  = startLeft.head(3);
    trajRightFootPose.block(0, 3, 3, 1) = startRight.head(3);

    // use orientation and position of groundframe to adapt to current pose:
    // the y-axsis always points in walking direction.
    Eigen::Matrix4f trajectoryGF = Bipedal::computeGroundFrame(trajLeftFootPose, trajRightFootPose, Bipedal::SUPPORT_BOTH);
    Eigen::Matrix4f currentGF    = Bipedal::computeGroundFrame(leftFootPose, rightFootPose, Bipedal::SUPPORT_BOTH);

    Eigen::Matrix4f adaptionFrame = currentGF * trajectoryGF.inverse();
    _mLFootTrajectory.topRows(3) = adaptionFrame.block(0, 0, 3, 3) * _mLFootTrajectory.topRows(3);
    _mRFootTrajectory.topRows(3) = adaptionFrame.block(0, 0, 3, 3) * _mRFootTrajectory.topRows(3);
    const Eigen::Vector3f& delta = adaptionFrame.block(0, 3, 3, 1);
    // matrix is stored in row-major order, this should be cache efficent
    _mLFootTrajectory.row(0).array() += delta.x();
    _mLFootTrajectory.row(1).array() += delta.y();
    _mLFootTrajectory.row(2).array() += delta.z();
    _mRFootTrajectory.row(0).array() += delta.x();
    _mRFootTrajectory.row(1).array() += delta.y();
    _mRFootTrajectory.row(2).array() += delta.z();
}

const Eigen::Matrix6Xf& FootstepPlaner::getLeftFootTrajectory() const
{
    BOOST_ASSERT(_bGenerated);
    return _mLFootTrajectory;
}

const Eigen::Matrix6Xf& FootstepPlaner::getRightFootTrajectory() const
{
    BOOST_ASSERT(_bGenerated);
    return _mRFootTrajectory;
}

const std::vector<Bipedal::SupportInterval>& FootstepPlaner::getSupportIntervals() const
{
    BOOST_ASSERT(_bGenerated);
    return _supportIntervals;
}

}

