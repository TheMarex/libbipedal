
#ifndef __PatternGeneratorScene_WINDOW_H_
#define __PatternGeneratorScene_WINDOW_H_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Obstacle.h>
#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTranslation.h>
#include <vector>

#include "PolynomialFootstepPlaner.h"

#include "ui_PatternGeneratorWindow.h"


class PatternGeneratorWindow : public QMainWindow
{
	Q_OBJECT
public:
	PatternGeneratorWindow(std::string &sRobotFile, Qt::WFlags flags = 0);
	~PatternGeneratorWindow();

	/*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
	int main();

public slots:
	/*! Closes the window and exits SoQt runloop. */
	void quit();
	/*!< Overriding the close event, so we know when the window was closed by the user. */
	void closeEvent(QCloseEvent *event);
	
	void collisionModel();
	void selectJoint(int nr);
	void jointValueChanged(int pos);
	void selectRNS(int nr);
	void performCoMIK();
	
	// handle UI responses
	void selectRobot();
	void resetSceneryAll();

	void updateStepPeriod();
	void updateStepLength();
	void updateDoubleSupportLength();
	void updateStepHeight();

	void calculateButtonPushed();

	void showFootstepPositions();
	void showFeetTrajectories();
	void showZMPTrajectory();
	void showCoMTrajectory();
	void showZMP();
	void showCoM();
	void showSupportPolygon();

protected:
	void loadRobot();
	void buildVisu();
	void getUIParameters();

	void setupUI();
	QString formatString(const char *s, float f);

	void updateJointBox();
	void updateRNSBox();
	void updateCoM();
	void updateSupportVisu();
	void updateTrajectoriesVisu();
	void updateZMPVisu();
	

	Ui::MainWindowStability UI;
	SoQtExaminerViewer *m_pExViewer; /*!< Viewer to display the 3D model of the robot and the environment. */

	Eigen::Vector2f m_CoMTarget;
		
	SoSeparator *sceneSep;
	SoSeparator *robotVisuSep;
	SoSeparator *comVisu;
	SoSeparator *comProjectionVisu;
	SoSeparator *comTargetVisu;
	SoSeparator *supportVisu;
	SoSeparator *_vFootstepPlaner;
	SoSeparator *_vZMPTrajectory;
	SoSeparator *_vCoMTrajectory;
	SoSeparator *_vZMP;

	//PolynomialFootstepPlaner *_footstepPlaner;
	
	VirtualRobot::RobotPtr robot;
	std::string robotFile;
	
	VirtualRobot::RobotNodeSetPtr currentRobotNodeSet;
	std::vector < VirtualRobot::RobotNodePtr > allRobotNodes;
	std::vector < VirtualRobot::RobotNodePtr > currentRobotNodes;
	std::vector < VirtualRobot::RobotNodeSetPtr > robotNodeSets;	
	
	VirtualRobot::RobotNodePtr currentRobotNode;
	
	bool useColModel;

    boost::shared_ptr<VirtualRobot::CoinVisualization> visualization;

	boost::shared_ptr<PolynomialFootstepPlaner> pFootStepPlaner;
};

#endif // __PatternGeneratorScene_WINDOW_H_
