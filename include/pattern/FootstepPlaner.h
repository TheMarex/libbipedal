#ifndef __Footstep_Planer_H_
#define __Footstep_Planer_H_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/MathTools.h>

#include <boost/shared_ptr.hpp>

#include "../utils/Kinematics.h"

class FootstepPlaner
{
public:
    FootstepPlaner();

    void generate();
    void setParameters(unsigned numberOfSteps,
                       unsigned sampleSize,
                       double stepLength,
                       double singleSupportPhase,
                       double doubleSupportPhase,
                       double stepHeight,
                       double angle=0, // in radians: >0 counter clockwise
                       double radius=1);

    void setRobotModel(const VirtualRobot::RobotPtr& pRobot,
                       const std::string& nameRightFoot = "RightLeg_BodyAnkle2",
                       const std::string& nameLeftFoot  = "LeftLeg_BodyAnkle2");

    VirtualRobot::RobotPtr getRobotModel() const { return _pRobot; };

    const Eigen::Matrix3Xf& getLeftFootPositions() const;
    const Eigen::Matrix3Xf& getRightFootPositions() const;

    const Eigen::Matrix3Xf& getLeftFootTrajectory() const;
    const Eigen::Matrix3Xf& getRightFootTrajectory() const;

    const std::vector<Kinematics::SupportInterval>& getSupportIntervals() const;

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
    void transformFootPositions();

    // data structures to save footstep positions and feet trajectories
    Eigen::Matrix3Xf _mLFootTrajectory;
    Eigen::Matrix3Xf _mRFootTrajectory;
    Eigen::Matrix3Xf _mLFootPositions;
    Eigen::Matrix3Xf _mRFootPositions;
    Eigen::Matrix3Xf _mLFootPositionsTransformed;
    Eigen::Matrix3Xf _mRFootPositionsTransformed;
    Eigen::Matrix3Xf _mLFootTrajectoryTransformed;
    Eigen::Matrix3Xf _mRFootTrajectoryTransformed;

    std::vector<Kinematics::SupportInterval> _supportIntervals;

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
    VirtualRobot::RobotPtr _pRobot;
    std::string _sLeftFootName;
    std::string _sRightFootName;

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

#endif
