#ifndef __Footstep_Planer_H_
#define __Footstep_Planer_H_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/MathTools.h>
#include <boost/shared_ptr.hpp>

class FootstepPlaner
{
public:
	FootstepPlaner();

    void generate(int numberOfSteps=5);
	virtual void setParameters(double stepLength, double stepPeriod, double doubleSupportPhase, double stepHeight, int sampleSize=100) = 0;

	void setRobotModel(VirtualRobot::RobotPtr pRobot,
		std::string nameRightFoot = "RightLeg_BodyAnkle2",
		std::string nameLeftFoot = "LeftLeg_BodyAnkle2");
    VirtualRobot::RobotPtr getRobotModel() {return _pRobot;};

	const Eigen::Matrix3Xf& getLeftFootPositions();
	const Eigen::Matrix3Xf& getRightFootPositions();

	const Eigen::Matrix3Xf& getLeftFootTrajectory();
	const Eigen::Matrix3Xf& getRightFootTrajectory();


	double getDSTime() {return _dDoubleSupportPhase;};
	double getSSTime() {return _dSingleSupportPhase;};
	bool isStartingWithLeftFoot() {return _bLeftFootFirst;};
	int getSamplesPerSecond() {return _iSampleSize;};
	float getStepHeight() {return _dStepHeight;};
    float getCoMHeight() {return _dCoMHeight;};
    Eigen::Vector3f getInitialCoMPosition() {return _mInitialCoMPosition;};
    void setInitialCoMPosition(Eigen::Vector3f v) {_mInitialCoMPosition=v;};
	VirtualRobot::MathTools::ConvexHull2DPtr getLeftFootConvexHull() {return cvLeft; };
	VirtualRobot::MathTools::ConvexHull2DPtr getRightFootConvexHull() {return cvRight; };

protected:
	virtual void computeFeetTrajectories(int numberOfSteps=5) = 0;

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

    Eigen::Vector3f _mInitialCoMPosition;

	VirtualRobot::MathTools::ConvexHull2DPtr cvRight;
	VirtualRobot::MathTools::ConvexHull2DPtr cvLeft;

	// parameters
	double _dStepLength;
	double _dStepWidth;
	double _dStepHeight;
	double _dStepPeriod;
    double _dCoMHeight;
	double _dSingleSupportPhase;
	double _dDoubleSupportPhase;
	double _iSampleSize;
	bool _bLeftFootFirst;

	// administrative bool values
	bool _bChangesMade;
	bool _bGenerated;

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
