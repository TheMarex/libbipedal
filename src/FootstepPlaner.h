#ifndef __Footstep_Planer_H_
#define __Footstep_Planer_H_

#include <VirtualRobot/VirtualRobot.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSwitch.h>
#include <Inventor/nodes/SoMaterial.h>

class FootstepPlaner
{
public:
	FootstepPlaner();
	~FootstepPlaner();

	virtual void generate(int numberOfSteps=5) = 0;
	virtual void setParameters(double stepLength, double stepPeriod, double doubleSupportPhase, double stepHeight, int sampleSize=100) = 0;

	void setRobotModel(VirtualRobot::RobotPtr pRobot, 
		std::string nameRightFoot = "RightLeg_BodyAnkle2", std::string nameLeftFoot = "LeftLeg_BodyAnkle2");
    VirtualRobot::RobotPtr getRobotModel() {return _pRobot;};

	SoSeparator* getVisualization();

	void showFootPositions(bool isVisible);
	void showFootTrajectories(bool isVisible);

	const Eigen::Matrix3Xf& getLeftFootPositions();
	const Eigen::Matrix3Xf& getRightFootPositions();

	const Eigen::Matrix3Xf& getLeftFootTrajectory();
	const Eigen::Matrix3Xf& getRightFootTrajectory();

	static void writeSceneGraphToFile(SoSeparator* node);

	double getDSTime() {return _dDoubleSupportPhase;};
	double getSSTime() {return _dSingleSupportPhase;};
	bool isStartingWithLeftFoot() {return _bLeftFootFirst;};
	int getSamplesPerSecond() {return _iSampleSize;};
	float getStepHeight() {return _dStepHeight;};
    float getCoMHeight() {return _dCoMHeight;};

    Eigen::Vector3f getInitialCoMPosition() {return _mInitialCoMPosition;};
    void setInitialCoMPosition(Eigen::Vector3f v) {_mInitialCoMPosition=v;};

	static void generateVisualizationDuplicatesFromTrajectories(SoSeparator* whereToInsert, 
		SoSeparator* whatToInsert, Eigen::Matrix3Xf &whereToTranslate);
    static void generateVisualizationForLineTrajectories(SoSeparator* whereToInsert, Eigen::Vector3f from, Eigen::Vector3f to, float rColor = 0.8f, float gColor = 0.1f, float bColor = 0.1f,  float scale=1000.0f);
    static void generateVisualizationForLineTrajectories(SoSeparator* whereToInsert, Eigen::Matrix3Xf positionList, float rColor = 0.8f, float gColor = 0.1f, float bColor = 0.1f,  float scale=1000.0f);

protected:
	void computeFeetShape();
	void buildVisualization();

//public:
	// data structures to save footstep positions and feet trajectories
	Eigen::Matrix3Xf _mLFootTrajectory;
	Eigen::Matrix3Xf _mRFootTrajectory;
protected:
	Eigen::Matrix3Xf _mLFootPositions;
	Eigen::Matrix3Xf _mRFootPositions;
	Eigen::Matrix3Xf _mLFootPositionsTransformed;
	Eigen::Matrix3Xf _mRFootPositionsTransformed;
	Eigen::Matrix3Xf _mLFootTrajectoryTransformed;
	Eigen::Matrix3Xf _mRFootTrajectoryTransformed;

    Eigen::Vector3f _mInitialCoMPosition;

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

	// ** visualization nodes **
	// visualization for the right foot (with border), without border, feet position and feet trajectory
	SoSeparator* _visuRightFoot;
	SoSeparator* _visuRightFootwoBorder;
	SoSeparator* _visuRightFootPositions;
	SoSeparator* _visuRightFootTrajectory;
	// visualization for the left foot (with border), without border, feet position and feet trajectory
	SoSeparator* _visuLeftFoot;
	SoSeparator* _visuLeftFootwoBorder;
	SoSeparator* _visuLeftFootPositions;
	SoSeparator* _visuLeftFootTrajectory;
	// root node for visualization
	SoSeparator* _visualization;
	// switches to enable or disable visualization of footstep positions and foot trajectories
	SoSwitch* _swPositions;
	SoSwitch* _swTrajectories;
};

#endif
