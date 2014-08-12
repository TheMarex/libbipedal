
#ifndef __ShowRobot_WINDOW_H_
#define __ShowRobot_WINDOW_H_

#include <VirtualRobot/VirtualRobot.h>

#include <Eigen/Dense>

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>

#include "../../include/bipedal.h"


#include "ui_Window.h"

class Window : public QMainWindow
{
	Q_OBJECT
public:
	Window(VirtualRobot::RobotPtr, Qt::WFlags flags = 0);
	~Window();

	/*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
	int main();

public slots:
	/*! Closes the window and exits SoQt runloop. */
	void quit();

	/*!< Overriding the close event, so we know when the window was closed by the user. */
	void closeEvent(QCloseEvent *event);

	void frameChanged(double);
	void updateCurrentRef();

	SoQtExaminerViewer* getExaminerViewer(){return viewer;};

protected:
	void setupUI();
	void updateIK();
	Ui::MainWindowShowRobot UI;
	SoQtExaminerViewer *viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

	SoSeparator *sceneSep;
	SoSeparator *robotSep;
	SoSeparator *robotInvSep;

	VirtualRobot::RobotPtr robot;
	VirtualRobot::RobotPtr robotInv;
	VirtualRobot::RobotNodeSetPtr nodes;

	ReferenceIKPtr referenceIK;
	std::vector<std::string> frameNames;
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> refFrames;

	bool computeIK;
};

#endif // __ShowRobot_WINDOW_H_
