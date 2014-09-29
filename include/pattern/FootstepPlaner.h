#ifndef __Footstep_Planer_H_
#define __Footstep_Planer_H_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/MathTools.h>

#include <boost/shared_ptr.hpp>

#include "../bipedal.h"
#include "../utils/Kinematics.h"

namespace Bipedal
{

class FootstepPlaner
{
public:
    FootstepPlaner(const VirtualRobot::RobotNodePtr& leftFootBody,
                   const VirtualRobot::RobotNodePtr& rightFootBody);

    void generate(const Eigen::Matrix4f& leftFootPose, const Eigen::Matrix4f& rightFootPose);
    void setParameters(unsigned numberOfSteps,
                       unsigned sampleSize,
                       double stepLength,
                       double singleSupportPhase,
                       double doubleSupportPhase,
                       double stepHeight,
                       double angle=0.0, // in radians: >0 counter clockwise
                       double radius=1.0);

    const Eigen::Matrix6Xf& getLeftFootTrajectory() const;
    const Eigen::Matrix6Xf& getRightFootTrajectory() const;

    const std::vector<Bipedal::SupportInterval>& getSupportIntervals() const;

    unsigned getSamplesPerSecond() const { return _iSampleSize; };
    double getStepHeight() const { return _dStepHeight; };
    double getStepLength() const { return _dStepHeight; };
    double getSSTime() const { return _dDoubleSupportPhase; };
    double getDSTime() const { return _dSingleSupportPhase; };

    VirtualRobot::MathTools::ConvexHull2DPtr getLeftFootConvexHull() const { return cvLeft; };
    VirtualRobot::MathTools::ConvexHull2DPtr getRightFootConvexHull() const { return cvRight; };

protected:
    virtual void computeFeetTrajectories() = 0;

    void computeFeetShape();

    // Units: In meter, not mm!
    void transformFootTrajectories(const Eigen::Matrix4f& leftFootPose, const Eigen::Matrix4f& rightFootPose);

    // data structures to save footstep positions and feet trajectories
    // We store x, y, z, roll, pitch and yaw
    Eigen::Matrix6Xf _mLFootTrajectory;
    Eigen::Matrix6Xf _mRFootTrajectory;

    std::vector<Bipedal::SupportInterval> _supportIntervals;

    VirtualRobot::MathTools::ConvexHull2DPtr cvRight;
    VirtualRobot::MathTools::ConvexHull2DPtr cvLeft;

    // parameters
    double _dStepLength;
    double _dStepWidth;
    double _dStepHeight;
    double _dStepPeriod;
    double _dSingleSupportPhase;
    double _dDoubleSupportPhase;
    double _dAngle;
    double _dRadius;
    unsigned _iSampleSize;
    unsigned _iNumberOfSteps;
    bool _bLeftFootFirst;

    // administrative bool values
    bool _bGenerated;
    bool _bParametersInitialized;

    // the robot model and the names of the foot segments
    VirtualRobot::RobotNodePtr _pLeftFootBody;
    VirtualRobot::RobotNodePtr _pRightFootBody;

    // starting points of robot, right-foot and left-foot
    Eigen::Vector2f _vRobotCenter;
    Eigen::Vector2f _vRightFootCenter;
    Eigen::Vector2f _vLeftFootCenter;

    // starting point of generated trajectories
    Eigen::Vector3f _vDeltaRightFoot;
    Eigen::Vector3f _vDeltaLeftFoot;

    // rotation for walking in the right direction
    Eigen::Matrix2f _mRotateWalking;
};

typedef boost::shared_ptr<FootstepPlaner> FootstepPlanerPtr;

}

#endif
