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
	
	void setRobotModel(VirtualRobot::RobotPtr pRobot, 
		std::string nameRightFoot = "RightLeg_BodyAnkle2", std::string nameLeftFoot = "LeftLeg_BodyAnkle2");

	SoSeparator* getVisualization();

	void showFootPositions(bool isVisible);
	void showFootTrajectories(bool isVisible);

	Eigen::Matrix3Xd const getLeftFootPositions() {return _mLFootPositions;};
	Eigen::Matrix3Xd const getRightFootPositions() {return _mRFootPositions;};
	static void writeSceneGraphToFile(SoSeparator* node);

protected:
	void computeFeetShape();
	void buildVisualization();
	static void generateVisualizationDuplicatesFromTrajectories(SoSeparator* whereToInsert, 
		SoSeparator* whatToInsert, Eigen::Matrix3Xd &whereToTranslate);

	Eigen::Matrix3Xd _mLFootTrajectory;
	Eigen::Matrix3Xd _mRFootTrajectory;
	Eigen::Matrix3Xd _mLFootPositions;
	Eigen::Matrix3Xd _mRFootPositions;

	bool _bChangesMade;
	bool _bGenerated;
	
	VirtualRobot::RobotPtr _pRobot;
	std::string _sLeftFootName;
	std::string _sRightFootName;

	// visualization for the right foot (with border), without border, feet position and feet trajectory
	SoSeparator* _visuRightFoot;
	SoSeparator* _visuRightFootwoBorder;
	SoSeparator* _visuRightFootPositions;
	SoSeparator* _visuRightFootTrajectory;
	// visualization for the left foot
	SoSeparator* _visuLeftFoot;
	SoSeparator* _visuLeftFootwoBorder;
	SoSeparator* _visuLeftFootPositions;
	SoSeparator* _visuLeftFootTrajectory;
	// complete visualization
	SoSeparator* _visualization;
	// switches to enable or disable visualizations
	SoSwitch* _swPositions;
	SoSwitch* _swTrajectories;
	// Transparent material
};

#endif
