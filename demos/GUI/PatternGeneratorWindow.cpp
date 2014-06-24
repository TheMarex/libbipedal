
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/XML/RobotIO.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h"
#include "VirtualRobot/CollisionDetection/CollisionChecker.h"
#include "VirtualRobot/BoundingBox.h"
#include "VirtualRobot/IK/CoMIK.h"

#include <QFileDialog>
#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoSphere.h>

#include "PatternGeneratorWindow.h"

#include "Utils/TrajectoryExporter.h"
#include "Utils/Kinematics.h"

#include <boost/make_shared.hpp>

#include <sstream>
using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

PatternGeneratorWindow::PatternGeneratorWindow(std::string &sRobotFile, Qt::WFlags flags)
:QMainWindow(NULL)
{
	VR_INFO << " start " << endl;
	pFootStepPlaner = boost::make_shared<PolynomialFootstepPlaner>();
	pZMPPreviewControl = boost::dynamic_pointer_cast<ZMPPlaner>(
		boost::make_shared<ZMPPreviewControl>()
	);
	pZMPVisu = boost::make_shared<ZMPVisualization>(pZMPPreviewControl);
	pFootVisu = boost::make_shared<FootVisualization>(pFootStepPlaner);
	pFootVisu->buildVisualization();

	robotFile = sRobotFile;
	useColModel = false;

	sceneSep = new SoSeparator();
	robotVisuSep = new SoSeparator();
	comVisu = new SoSeparator();	
	comProjectionVisu = new SoSeparator();
	comTargetVisu = new SoSeparator();
	supportVisu = new SoSeparator();

	_vFootstepPlaner = new SoSeparator();
	_vFootstepPlaner->ref();
	_vZMPTrajectory = new SoSeparator();
	_vZMPTrajectory->ref();

	_vCoMTrajectory = new SoSeparator();
	_vZMP = new SoSeparator();

	sceneSep->ref();
	sceneSep->addChild(robotVisuSep);
	sceneSep->addChild(comVisu);
	sceneSep->addChild(comProjectionVisu);
	sceneSep->addChild(comTargetVisu);
	sceneSep->addChild(supportVisu);
	sceneSep->addChild(_vFootstepPlaner);
	sceneSep->addChild(_vZMPTrajectory);
	_vFootstepPlaner->addChild(pFootVisu->getVisualization());
	//std::cout << "Foostep Planer: [" << pFootStepPlaner.get() << "], visualization: [" << pFootStepPlaner->getVisualization() << "]" << std::endl;
	_vZMPTrajectory->addChild(pZMPVisu->getVisualization());

	sceneSep->addChild(_vCoMTrajectory);
	sceneSep->addChild(_vZMP);


	MathTools::Plane p =  MathTools::getFloorPlane();
	sceneSep->addChild(CoinVisualizationFactory::CreatePlaneVisualization(p.p,p.n,10000.0f,0.0f));

	m_CoMTarget = Eigen::Vector2f::Zero();
	
	setupUI();
	
	loadRobot();

	m_pExViewer->viewAll();
}


PatternGeneratorWindow::~PatternGeneratorWindow()
{
	_vFootstepPlaner->unref();
	_vZMPTrajectory->unref();
	sceneSep->unref();
}


void PatternGeneratorWindow::setupUI()
{
	 UI.setupUi(this);
	 m_pExViewer = new SoQtExaminerViewer(UI.frameViewer,"",TRUE,SoQtExaminerViewer::BUILD_POPUP);

	// setup
	m_pExViewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
	m_pExViewer->setAccumulationBuffer(true);
//#ifdef WIN32
//#ifndef _DEBUG
	m_pExViewer->setAntialiasing(true, 4);
//#endif
//#endif
	m_pExViewer->setGLRenderAction(new SoLineHighlightRenderAction);
	m_pExViewer->setTransparencyType(SoGLRenderAction::BLEND);
	m_pExViewer->setFeedbackVisibility(true);
	m_pExViewer->setSceneGraph(sceneSep);
	m_pExViewer->viewAll();

	connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectRobot()));
	connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
	connect(UI.pushButtonExport, SIGNAL(clicked()), this, SLOT(exportTrajectory()));
	
	connect(UI.lineEditStepPeriod, SIGNAL(editingFinished()), this, SLOT(updateStepPeriod()));
	connect(UI.lineEditStepLength, SIGNAL(editingFinished()), this, SLOT(updateStepLength()));
	connect(UI.lineEditDSLength, SIGNAL(editingFinished()), this, SLOT(updateDoubleSupportLength()));
	connect(UI.lineEditStepHeight, SIGNAL(editingFinished()), this, SLOT(updateStepHeight()));

	connect(UI.pushButtonCalculate, SIGNAL(clicked()), this, SLOT(calculateButtonPushed()));

	connect(UI.checkBoxFootstepPositions, SIGNAL(clicked()), this, SLOT(showFootstepPositions()));
	connect(UI.checkBoxFeetTrajectories, SIGNAL(clicked()), this, SLOT(showFeetTrajectories()));
	connect(UI.checkBoxZMPTrajectory, SIGNAL(clicked()), this, SLOT(showZMPTrajectory()));
	connect(UI.checkBoxCoMTrajectory, SIGNAL(clicked()), this, SLOT(showCoMTrajectory()));
	connect(UI.checkBoxZMP, SIGNAL(clicked()), this, SLOT(showZMP()));
    connect(UI.checkBoxCoM, SIGNAL(clicked()), this, SLOT(showCoM()));
    connect(UI.checkBoxSupportPolygon, SIGNAL(clicked()), this, SLOT(showSupportPolygon()));

    connect(UI.frameSlider, SIGNAL(valueChanged(int)), this, SLOT(trajectorySliderValueChanged(int)));
	
	UI.lineEditDSLength->setValidator(new QDoubleValidator());
	UI.lineEditStepHeight->setValidator(new QDoubleValidator());
	UI.lineEditStepLength->setValidator(new QDoubleValidator());
	UI.lineEditStepPeriod->setValidator(new QDoubleValidator());

}

QString PatternGeneratorWindow::formatString(const char *s, float f)
{
	QString str1(s);
	if (f>=0)
		str1 += " ";
	if (fabs(f)<1000)
		str1 += " ";
	if (fabs(f)<100)
		str1 += " ";
	if (fabs(f)<10)
		str1 += " ";
	QString str1n;
	str1n.setNum(f,'f',3);
	str1 = str1 + str1n;
	return str1;
}

void PatternGeneratorWindow::closeEvent(QCloseEvent *event)
{
	quit();
	QMainWindow::closeEvent(event);
}

void PatternGeneratorWindow::buildVisu()
{
	if (!robot)
		return;
	robotVisuSep->removeAllChildren();
//	useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
	SceneObject::VisualizationType colModel = SceneObject::Full; //(UI.checkBoxColModel->isChecked())?SceneObject::Collision:SceneObject::Full;
	visualization = robot->getVisualization<CoinVisualization>(colModel);
	SoNode* visualisationNode = NULL;
	if (visualization)
		visualisationNode = visualization->getCoinVisualization();

	if (visualisationNode)
		robotVisuSep->addChild(visualisationNode);

	comVisu->removeAllChildren();
	comProjectionVisu->removeAllChildren();
	comTargetVisu->removeAllChildren();
	if (UI.checkBoxCoM->isChecked())
	{
		SoMatrixTransform *m = new SoMatrixTransform();
		comVisu->addChild(m);
		SoMaterial *material = new SoMaterial;
		material->diffuseColor.setValue(1.0f,0.2f,0.2f);
		comVisu->addChild(material);
		SoSphere *s = new SoSphere;
		s->radius.setValue(0.0300f);
		SoCube *c = new SoCube;
		c->width.setValue(0.050f);
		c->height.setValue(0.050f);
		c->depth.setValue(0.050f);
		comVisu->addChild(c);
		comVisu->addChild(s);

		m = new SoMatrixTransform();
		comProjectionVisu->addChild(m);
		material = new SoMaterial;
		material->diffuseColor.setValue(0.2f,0.2f,1.0f);
		comProjectionVisu->addChild(material);
		s = new SoSphere;
		s->radius.setValue(0.030f);
		c = new SoCube;
		c->width.setValue(0.050f);
		c->height.setValue(0.050f);
		c->depth.setValue(0.050f);
		comProjectionVisu->addChild(c);
		comProjectionVisu->addChild(s);

		m = new SoMatrixTransform();
		comTargetVisu->addChild(m);
		material = new SoMaterial;
		material->diffuseColor.setValue(0.2f,0.2f,0.2f);
		comTargetVisu->addChild(material);
		s = new SoSphere;
		s->radius.setValue(0.0300f);
		c = new SoCube;
		c->width.setValue(0.050f);
		c->height.setValue(0.050f);
		c->depth.setValue(0.050f);
		comTargetVisu->addChild(c);
		comTargetVisu->addChild(s);
		updateCoM();
	}

	updateSupportVisu();
	updateTrajectoriesVisu();
	updateZMPVisu();

	m_pExViewer->scheduleRedraw();

}

void PatternGeneratorWindow::getUIParameters() 
{
	QString value;
	double dStepPeriod, dStepLength, dDSPhase, dHeight;
	value = UI.lineEditStepPeriod->text();
	dStepPeriod = value.toDouble();
	value = UI.lineEditStepLength->text();
	dStepLength = value.toDouble();
	value = UI.lineEditDSLength->text();
	dDSPhase = value.toDouble();
	value = UI.lineEditStepHeight->text();
	dHeight = value.toDouble();
	pFootStepPlaner->setParameters(dStepLength, dStepPeriod, dDSPhase, dHeight);
    pFootStepPlaner->generate(5);
	pZMPPreviewControl->setFootstepPlaner(pFootStepPlaner);
    if (robot!=0) {
        std::cout << "Initial CoM Position set!" << std::endl;
        Eigen::Vector3f com = robot->getCoMLocal();
        pFootStepPlaner->setInitialCoMPosition(com);
    }

    pZMPPreviewControl->computeReference();

	VirtualRobot::RobotNodePtr leftArm = robot->getRobotNode("LeftArm_Joint3");
	robot->setJointValue(leftArm, 0.3);
	VirtualRobot::RobotNodePtr rightArm = robot->getRobotNode("RightArm_Joint3");
	robot->setJointValue(rightArm, -0.3);

	Kinematics::computeWalkingTrajectory(robot,
		pZMPPreviewControl->getCoMTrajectory(),
		pFootStepPlaner->getRightFootTrajectory(),
		pFootStepPlaner->getLeftFootTrajectory(),
		trajectoy);

	pZMPVisu->buildCoMVisualization();
	pZMPVisu->buildComputedZMPVisualization();
	pZMPVisu->buildReferenceZMPVisualization();
	pFootVisu->buildVisualization();

    // Update slider
    int numSteps = trajectoy.cols();
    std::cout << "Walking trajectory has " << numSteps << " steps" << std::endl;
    UI.frameSlider->setRange(0, numSteps - 1);
    UI.frameSlider->setEnabled(true);

	UI.pushButtonExport->setEnabled(true);

    // The trajectory computation changes the robot configuration => Reset
    resetSceneryAll();
}

void PatternGeneratorWindow::updateCoM()
{
	if (!comVisu || comVisu->getNumChildren()==0)
		return;

	// Draw CoM
	Eigen::Matrix4f globalPoseCoM;	
	globalPoseCoM.setIdentity();
    globalPoseCoM.block(0,3,3,1) = robot->getRobotNodeSet("ColModelAll")->getCoM();
    /*if (currentRobotNodeSet)
	{
		globalPoseCoM.block(0,3,3,1) = currentRobotNodeSet->getCoM();
	} else if (robot)
	{
        globalPoseCoM.block(0,3,3,1) = robot->getCoMLocal();
    }*/
	SoMatrixTransform *m = dynamic_cast<SoMatrixTransform *>(comVisu->getChild(0));
	if (m)
	{
		SbMatrix ma(reinterpret_cast<SbMat*>(globalPoseCoM.data()));
        // mm -> m
        ma[3][0] *= 0.001f;
        ma[3][1] *= 0.001f;
        ma[3][2] *= 0.001f;
		m->matrix.setValue(ma);
	}

	// Draw CoM projection
    m = dynamic_cast<SoMatrixTransform *>(comProjectionVisu->getChild(0));
    //if(currentRobotNodeSet && comProjectionVisu && comProjectionVisu->getNumChildren() > 0)
    if (m)
	{
		globalPoseCoM(2, 3) = 0;
		m = dynamic_cast<SoMatrixTransform *>(comProjectionVisu->getChild(0));
		if(m)
		{
			SbMatrix ma(reinterpret_cast<SbMat*>(globalPoseCoM.data()));
            // mm -> m
            ma[3][0] *= 0.001f;
            ma[3][1] *= 0.001f;
            ma[3][2] *= 0.001f;
			m->matrix.setValue(ma);
		}
	}
}

// changes on Visualization of SupportPolygon or 
void PatternGeneratorWindow::updateSupportVisu()
{
	supportVisu->removeAllChildren();
	MathTools::Plane p =  MathTools::getFloorPlane();

	if (UI.checkBoxSupportPolygon->isChecked())
	{
		std::vector< CollisionModelPtr > colModels =  robot->getCollisionModels();
		CollisionCheckerPtr colChecker = CollisionChecker::getGlobalCollisionChecker();
		std::vector< CollisionModelPtr >::iterator i = colModels.begin();

		std::vector< MathTools::ContactPoint > points;
		while (i!=colModels.end())
		{
			colChecker->getContacts(p, *i, points, 5.0f);
			i++;
		}

		std::vector< Eigen::Vector2f > points2D;
		for (size_t u=0;u<points.size();u++)
		{
			Eigen::Vector2f pt2d = MathTools::projectPointToPlane2D(points[u].p,p);
			points2D.push_back(pt2d);
		}

		MathTools::ConvexHull2DPtr cv = MathTools::createConvexHull2D(points2D);
		SoSeparator* visu = CoinVisualizationFactory::CreateConvexHull2DVisualization(cv,p,
			VisualizationFactory::Color::Blue(),VisualizationFactory::Color::Black(),6.0f,Eigen::Vector3f(0,0,2.0f));
		supportVisu->addChild(visu);
	}

	pFootVisu->showFootPositions( UI.checkBoxFootstepPositions->isChecked() );
}

// changes on ZMP or Feet Trajectory Checkboxes
void PatternGeneratorWindow::updateTrajectoriesVisu() 
{
	// check for trajectories and enable/disable them
	pFootVisu->showFootTrajectories( UI.checkBoxFeetTrajectories->isChecked() );

	pZMPVisu->showReference(UI.checkBoxZMPTrajectory->isChecked());

	pZMPVisu->showRealZMP(UI.checkBoxZMP->isChecked());

	pZMPVisu->showCoM(UI.checkBoxCoMTrajectory->isChecked());
}

void PatternGeneratorWindow::updateZMPVisu() 
{
}

void PatternGeneratorWindow::updateRNSBox()
{
	std::vector < VirtualRobot::RobotNodeSetPtr > allRNS;
	robot->getRobotNodeSets(allRNS);
	robotNodeSets.clear();

	for (unsigned int i=0;i<allRobotNodes.size();i++)
	{
		allRobotNodes[i]->showBoundingBox(false);
	}
}

int PatternGeneratorWindow::main()
{
	SoQt::show(this);
	SoQt::mainLoop();
	return 0;
}

void PatternGeneratorWindow::quit()
{
	std::cout << "PatternGeneratorWindow: Closing" << std::endl;
	this->close();
	SoQt::exitMainLoop();
}

void PatternGeneratorWindow::loadRobot()
{
	robotVisuSep->removeAllChildren();
	cout << "Loading Scene from " << robotFile << endl;

	try
	{
		robot = RobotIO::loadRobot(robotFile);
	}
	catch (VirtualRobotException &e)
	{
		cout << " ERROR while creating robot" << endl;
		cout << e.what();
		return;
	}

	if (!robot)
	{
		cout << " ERROR while creating robot" << endl;
		return;
	}

	// get nodes
	robot->getRobotNodes(allRobotNodes);

	// build visualization
	if (pFootStepPlaner)
		pFootStepPlaner->setRobotModel(robot);
	buildVisu();
	m_pExViewer->viewAll();
}

/*
 *
 *	hande UI responses
 *
 */

void PatternGeneratorWindow::selectRobot()
{
	QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), QString(), tr("XML Files (*.xml)"));
	robotFile = std::string(fi.toAscii());
	loadRobot();
}

void PatternGeneratorWindow::exportTrajectory()
{
	QString fileNameQ = QFileDialog::getSaveFileName(this,
			tr("Save MMM Motion"), "",
			tr("MMM motion XML file (*.xml)"));

	const Eigen::Matrix3Xf leftFootTrajectory = pFootStepPlaner->getLeftFootTrajectory();
	const Eigen::Matrix3Xf comTrajectory = pZMPPreviewControl->getCoMTrajectory();
	const Eigen::Matrix3Xf comVel = pZMPPreviewControl->getCoMVelocity();
	const Eigen::Matrix3Xf comAcc = pZMPPreviewControl->getCoMAcceleration();
	const Eigen::Matrix2Xf zmpTrajectory = pZMPPreviewControl->getComputedZMPTrajectory();
	const Eigen::Matrix2Xf refZMPTrajectory = pZMPPreviewControl->getReferenceZMPTrajectory();
	const std::vector<ZMPPlaner::SupportPhase> phase = pZMPPreviewControl->getSupportPhases();

	Eigen::Matrix3Xf relCom;
    Kinematics::computeRelativeCoMTrajectory(robot,
        leftFootTrajectory,
        robot->getRobotNode("Waist"),
        robot->getRobotNodeSet("Left2RightLeg"),
        trajectoy,
        comTrajectory,
        relCom
    );

	TrajectoryExporter exporter(robot,
		robotFile,
		trajectoy,
		leftFootTrajectory,
		relCom,
        comVel,
        comAcc,
		zmpTrajectory,
		refZMPTrajectory,
        phase,
		1.0f / pFootStepPlaner->getSamplesPerSecond()
	);

	exporter.exportToMMM(fileNameQ.toStdString());
}

// set all joint values to 0
void PatternGeneratorWindow::resetSceneryAll()
{
	if (!robot)
		return;

    Eigen::Matrix4f pose = robot->getGlobalPose();
    pose.block(0,3,3,1) = Eigen::Vector3f(0, 0, 0);
    robot->setGlobalPose(pose);

	std::vector< RobotNodePtr > nodes;
	robot->getRobotNodes(nodes);
	std::vector<float> jv(nodes.size(),0.0f);
	robot->setJointValues(nodes,jv);
}

// update Step Period
void PatternGeneratorWindow::updateStepPeriod()
{
	cout << "Step Period updated!" << endl;
}

void PatternGeneratorWindow::updateStepLength()
{
	cout << "Step Length updated!" << endl;
}

void PatternGeneratorWindow::updateDoubleSupportLength()
{
	cout << "Double Support Length updated!" << endl;
}

void PatternGeneratorWindow::updateStepHeight()
{
	cout << "Step Height updated!" << endl;
}

void PatternGeneratorWindow::calculateButtonPushed()
{
	cout << "calculate Button pressed!" << endl;
	getUIParameters();
    //pFootStepPlaner->generate();
}

void PatternGeneratorWindow::showFootstepPositions()
{
	cout << "Footstep Positions clicked!" << endl;
	updateSupportVisu();
	m_pExViewer->scheduleRedraw();
}

void PatternGeneratorWindow::showFeetTrajectories()
{
	cout << "Feet Trajectories clicked!" << endl;
	updateTrajectoriesVisu();
	m_pExViewer->scheduleRedraw();
}

void PatternGeneratorWindow::showZMPTrajectory()
{
	cout << "ZMP Trajectory clicked!" << endl;
	updateTrajectoriesVisu();
	m_pExViewer->scheduleRedraw();
	//pFootStepPlaner->writeSceneGraphToFile(pFootStepPlaner->getVisualization());
}

void PatternGeneratorWindow::showCoMTrajectory()
{
	cout << "CoM Trajectory clicked!" << endl;
	updateTrajectoriesVisu();
    m_pExViewer->scheduleRedraw();
}

void PatternGeneratorWindow::showZMP()
{
    cout << "showZMP clicked!" << endl;
    updateTrajectoriesVisu();
    m_pExViewer->scheduleRedraw();
}

void PatternGeneratorWindow::showCoM()
{
	cout << "showCoM clicked!" << endl;
	if (!robot)
		return;
	buildVisu();
	updateCoM();
}

void PatternGeneratorWindow::showSupportPolygon()
{
	cout << "showSupportPolygon!" << endl;
	if (!robot)
		return;
    updateSupportVisu();
}

void PatternGeneratorWindow::trajectorySliderValueChanged(int value)
{
    if(!robot)
    {
        return;
    }

    RobotNodeSetPtr nodeSet = robot->getRobotNodeSet("Left2RightLeg");

    Eigen::Matrix3Xf leftFootTrajectory = pFootStepPlaner->getLeftFootTrajectory();
    Eigen::Matrix4f leftFootPose = nodeSet->getKinematicRoot()->getGlobalPose();

    leftFootPose.block(0,3,3,1) = 1000 * leftFootTrajectory.col(value);
    robot->setGlobalPose(leftFootPose);

    nodeSet->setJointValues(trajectoy.col(value));
    updateCoM();
	updateSupportVisu();
}

