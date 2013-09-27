#ifndef __Footstep_Planer_H_
#define __Footstep_Planer_H_

#include <VirtualRobot/VirtualRobot.h>
#include <Inventor/nodes/SoSeparator.h>

class FootstepPlaner
{
public:
	FootstepPlaner();
	~FootstepPlaner();

	virtual void generate(int numberOfSteps=5) = 0;
	
	void setRobotModel(VirtualRobot::RobotPtr pRobot, std::string nameRightFoot = "RightLeg_BodyAnkle2", std::string nameLeftFoot = "LeftLeg_BodyAnkle2");

	void addToVisualization(SoSeparator* root);

	Eigen::Matrix3Xd const getLeftFootPositions() {return _mLFootPositions;};
	Eigen::Matrix3Xd const getRightFootPositions() {return _mRFootPositions;};

protected:
	void computeFeetShape();
	void buildVisualization();
	static void generateVisualizationDuplicatesFromTrajectories(SoSeparator* whereToInsert, SoSeparator* whatToInsert, Eigen::Matrix3Xd &whereToTranslate);

	Eigen::Matrix3Xd _mLFootTrajectory;
	Eigen::Matrix3Xd _mRFootTrajectory;
	Eigen::Matrix3Xd _mLFootPositions;
	Eigen::Matrix3Xd _mRFootPositions;

	bool _bChangesMade;
	bool _bGenerated;
	
	VirtualRobot::RobotPtr _pRobot;
	std::string _sLeftFootName;
	std::string _sRightFootName;

	// visualization for the right foot
	SoSeparator* _visuRightFoot;
	SoSeparator* _visuRightFootPositions;
	SoSeparator* _visuRightFootTrajectory;
	// visualization for the left foot
	SoSeparator* _visuLeftFoot;
	SoSeparator* _visuLeftFootPositions;
	SoSeparator* _visuLeftFootTrajectory;
	// complete visualization
	SoSeparator* _visualization;
};

#endif
