
#include "PatternGeneratorWindow.h"
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

#include <sstream>
using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

PatternGeneratorWindow::PatternGeneratorWindow(std::string &sRobotFile, Qt::WFlags flags)
:QMainWindow(NULL)
{
	VR_INFO << " start " << endl;

	robotFile = sRobotFile;
	useColModel = false;
	sceneSep = new SoSeparator;
	sceneSep->ref();
	robotVisuSep = new SoSeparator;
	sceneSep->addChild(robotVisuSep);

	comVisu = new SoSeparator;	
	sceneSep->addChild(comVisu);
	comProjectionVisu = new SoSeparator;
	sceneSep->addChild(comProjectionVisu);
	comTargetVisu = new SoSeparator;
	sceneSep->addChild(comTargetVisu);
	supportVisu = new SoSeparator;
	sceneSep->addChild(supportVisu);

	MathTools::Plane p =  MathTools::getFloorPlane();
	sceneSep->addChild(CoinVisualizationFactory::CreatePlaneVisualization(p.p,p.n,10000.0f,0.0f));

	m_CoMTarget = Eigen::Vector2f::Zero();
	
	setupUI();
	
	loadRobot();

	m_pExViewer->viewAll();
}


PatternGeneratorWindow::~PatternGeneratorWindow()
{
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


//	connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(collisionModel()));
//	connect(UI.checkBoxSupportPolygon, SIGNAL(clicked()), this, SLOT(showSupportPolygon()));
//	connect(UI.comboBoxRNS, SIGNAL(activated(int)), this, SLOT(selectRNS(int)));
//	connect(UI.comboBoxJoint, SIGNAL(activated(int)), this, SLOT(selectJoint(int)));
//	connect(UI.horizontalSliderPos, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));
//	connect(UI.horizontalSliderPos, SIGNAL(sliderReleased()), this, SLOT(showSupportPolygon()));
//	connect(UI.sliderX, SIGNAL(valueChanged(int)), this, SLOT(comTargetMovedX(int)));
//	connect(UI.sliderY, SIGNAL(valueChanged(int)), this, SLOT(comTargetMovedY(int)));
	
	connect(UI.checkBoxCoM, SIGNAL(clicked()), this, SLOT(showCoM()));
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




void PatternGeneratorWindow::collisionModel()
{
	if (!robot)
		return;
	buildVisu();
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

	m_pExViewer->scheduleRedraw();

}

void PatternGeneratorWindow::updateCoM()
{
	if (!comVisu || comVisu->getNumChildren()==0)
		return;

	// Draw CoM
	Eigen::Matrix4f globalPoseCoM;	
	globalPoseCoM.setIdentity();
	if (currentRobotNodeSet)
	{
		globalPoseCoM.block(0,3,3,1) = currentRobotNodeSet->getCoM();
	} else if (robot)
	{
		globalPoseCoM.block(0,3,3,1) = robot->getCoM();
	}
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
	if(currentRobotNodeSet && comProjectionVisu && comProjectionVisu->getNumChildren() > 0)
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


void PatternGeneratorWindow::updateSupportVisu()
{
	supportVisu->removeAllChildren();
	//if (UI.checkBoxSupportPolygon->isChecked())
	//{
	//	/*SoMaterial *material = new SoMaterial;
	//	material->diffuseColor.setValue(1.0f,0.2f,0.2f);
	//	supportVisu->addChild(material);*/

	//	MathTools::Plane p =  MathTools::getFloorPlane();
	//	
	//	//supportVisu->addChild(CoinVisualizationFactory::CreatePlaneVisualization(p.p,p.n,100000.0f,0.5f));
	//	/*RobotNodeSetPtr rns = robot->getRobotNodeSet("TorsoRightArm");
	//	RobotNodePtr rn = robot->getRobotNode("Wrist1");
	//	CollisionModelPtr colModel = rn->getCollisionModel();
	//	colChecker->getContacts(p, colModel, points, 5.0f);*/


	//	std::vector< CollisionModelPtr > colModels =  robot->getCollisionModels();
	//	CollisionCheckerPtr colChecker = CollisionChecker::getGlobalCollisionChecker();
	//	std::vector< CollisionModelPtr >::iterator i = colModels.begin();

	//	std::vector< MathTools::ContactPoint > points;
	//	while (i!=colModels.end())
	//	{
	//		colChecker->getContacts(p, *i, points, 5.0f);
	//		i++;
	//	}

	//	std::vector< Eigen::Vector2f > points2D;
	//	//MathTools::Plane plane2(Eigen::Vector3f(0,0,0),Eigen::Vector3f(0,1.0f,0));
	//	for (size_t u=0;u<points.size();u++)
	//	{
	//		
	//		Eigen::Vector2f pt2d = MathTools::projectPointToPlane2D(points[u].p,p);
	//		points2D.push_back(pt2d);
	//	}

	//	MathTools::ConvexHull2DPtr cv = MathTools::createConvexHull2D(points2D);
	//	SoSeparator* visu = CoinVisualizationFactory::CreateConvexHull2DVisualization(cv,p,VisualizationFactory::Color::Blue(),VisualizationFactory::Color::Black(),6.0f,Eigen::Vector3f(0,0,2.0f));
	//	supportVisu->addChild(visu);
	//}
}

void PatternGeneratorWindow::updateRNSBox()
{
//	UI.comboBoxRNS->clear();
//	UI.comboBoxRNS->addItem(QString("<All>"));

	std::vector < VirtualRobot::RobotNodeSetPtr > allRNS;
	robot->getRobotNodeSets(allRNS);
	robotNodeSets.clear();	

	for (size_t i=0;i<allRNS.size();i++)
	{
		//if (allRNS[i]->isKinematicChain())
		//{
			//VR_INFO << " RNS <" << allRNS[i]->getName() << "> is a valid kinematic chain" << endl;
//			robotNodeSets.push_back(allRNS[i]);
//			UI.comboBoxRNS->addItem(QString(allRNS[i]->getName().c_str()));
		/*} else
		{
			VR_INFO << " RNS <" << allRNS[i]->getName() << "> is not a valid kinematic chain" << endl;
		}*/
	}
		

	for (unsigned int i=0;i<allRobotNodes.size();i++)
	{
		allRobotNodes[i]->showBoundingBox(false);
	}

	
	updateJointBox();
}

void PatternGeneratorWindow::selectRNS(int nr)
{
	currentRobotNodeSet.reset();
	cout << "Selecting RNS nr " << nr << endl;
	if (nr<=0)
	{
		// all joints
		currentRobotNodes = allRobotNodes;
	} else
	{
		nr--;
		if (nr>=(int)robotNodeSets.size())
			return;
		currentRobotNodeSet = robotNodeSets[nr];
		currentRobotNodes = currentRobotNodeSet->getAllRobotNodes();

		// Set CoM target to current CoM position
		Eigen::Vector3f com = currentRobotNodeSet->getCoM();
		//UI.sliderX->setValue(com(0));
		//UI.sliderY->setValue(com(1));
	}
	updateJointBox();
	selectJoint(0);
	updateCoM();
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


void PatternGeneratorWindow::updateJointBox()
{
	//UI.comboBoxJoint->clear();

	//for (unsigned int i=0;i<currentRobotNodes.size();i++)
	//{
	//	UI.comboBoxJoint->addItem(QString(currentRobotNodes[i]->getName().c_str()));
	//}
	selectJoint(0);
}

void PatternGeneratorWindow::jointValueChanged(int pos)
{	
	//int nr = UI.comboBoxJoint->currentIndex();
	//if (nr<0 || nr>=(int)currentRobotNodes.size())
	//	return;
	//float fPos = currentRobotNodes[nr]->getJointLimitLo() + (float)pos / 1000.0f * (currentRobotNodes[nr]->getJointLimitHi() - currentRobotNodes[nr]->getJointLimitLo());
	//robot->setJointValue(currentRobotNodes[nr],fPos);
	//UI.lcdNumberJointValue->display((double)fPos);

	updateCoM();

	/*RobotNodePtr p = robot->getRobotNode("Foot2 L");
	if (p)
	{
		BoundingBox bbox = p->getCollisionModel()->getBoundingBox(true);
		supportVisu->addChild(CoinVisualizationFactory::CreateBBoxVisualization(bbox));
	}*/
	// show bbox
	if (currentRobotNodeSet)
	{
		for (unsigned int i=0;i<currentRobotNodeSet->getSize();i++)
		{
			currentRobotNodeSet->getNode(i)->showBoundingBox(true,true);
		}
	}
}

void PatternGeneratorWindow::selectJoint(int nr)
{
	currentRobotNode.reset();
	cout << "Selecting Joint nr " << nr << endl;
	if (nr<0 || nr>=(int)currentRobotNodes.size())
		return;
	currentRobotNode = currentRobotNodes[nr];
	currentRobotNode->print();
	float mi = currentRobotNode->getJointLimitLo();
	float ma = currentRobotNode->getJointLimitHi();
	QString qMin = QString::number(mi);
	QString qMax = QString::number(ma);
	//UI.labelMinPos->setText(qMin);
	//UI.labelMaxPos->setText(qMax);
	//float j = currentRobotNode->getJointValue();
	//UI.lcdNumberJointValue->display((double)j);
	//if (fabs(ma-mi)>0 && (currentRobotNode->isTranslationalJoint() || currentRobotNode->isRotationalJoint()) )
	//{
	//	UI.horizontalSliderPos->setEnabled(true);
	//	int pos = (int)((j-mi)/(ma-mi) * 1000.0f);
	//	UI.horizontalSliderPos->setValue(pos);
	//} else
	//{
	//	UI.horizontalSliderPos->setValue(500);
	//	UI.horizontalSliderPos->setEnabled(false);
	//}
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
	updateRNSBox();
	selectRNS(0);
	if (allRobotNodes.size()==0)
		selectJoint(-1);
	else
		selectJoint(0);


	// build visualization
	buildVisu();
	m_pExViewer->viewAll();
}

void PatternGeneratorWindow::performCoMIK()
{
	if(!currentRobotNodeSet)
		return;

	CoMIK comIK(currentRobotNodeSet,currentRobotNodeSet);
	comIK.setGoal(m_CoMTarget);
	if(!comIK.solveIK(0.3f,0,20))
		std::cout << "IK solver did not succeed" << std::endl;
	updateCoM();
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

// set all joint values to 0
void PatternGeneratorWindow::resetSceneryAll()
{
	if (!robot)
		return;
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
	FootstepPlaner *planer = new PolynomialFootstepPlaner();
	planer->generate();
	delete planer;
}

void PatternGeneratorWindow::showFootstepPositions()
{
	cout << "Step Height updated!" << endl;
}

void PatternGeneratorWindow::showFeetTrajectories()
{
	cout << "Step Height updated!" << endl;
}

void PatternGeneratorWindow::showZMPTrajectory()
{
	cout << "ShowZMPTrajectory!" << endl;
}

void PatternGeneratorWindow::showCoMTrajectory()
{
	cout << "showCoMTrajectory!" << endl;
}

void PatternGeneratorWindow::showZMP()
{
	cout << "showZMP!" << endl;
}

void PatternGeneratorWindow::showCoM()
{
	cout << "showCoM!" << endl;
	if (!robot)
		return;
	buildVisu();
}

void PatternGeneratorWindow::showSupportPolygon()
{
	cout << "showSupportPolygon!" << endl;
	if (!robot)
		return;
	updateSupportVisu();
}




/*
void PatternGeneratorWindow::comTargetMovedX(int value)
{
	if(!currentRobotNodeSet)
		return;

	Eigen::Matrix4f T;
	T.setIdentity();

	m_CoMTarget(0) = value;
	T.block(0, 3, 2, 1) = m_CoMTarget;

	if(comTargetVisu && comTargetVisu->getNumChildren() > 0)
	{
		SoMatrixTransform *m = dynamic_cast<SoMatrixTransform *>(comTargetVisu->getChild(0));
		if(m)
		{
			SbMatrix ma(reinterpret_cast<SbMat*>(T.data()));
            // mm -> m
            ma[3][0] *= 0.001f;
            ma[3][1] *= 0.001f;
            ma[3][2] *= 0.001f;
			m->matrix.setValue(ma);
		}
	}

	performCoMIK();
}*/

/*
void PatternGeneratorWindow::comTargetMovedY(int value)
{
	Eigen::Matrix4f T;
	T.setIdentity();

	m_CoMTarget(1) = value;
	T.block(0, 3, 2, 1) = m_CoMTarget;

	if(comTargetVisu && comTargetVisu->getNumChildren() > 0)
	{
		SoMatrixTransform *m = dynamic_cast<SoMatrixTransform *>(comTargetVisu->getChild(0));
		if(m)
		{
			SbMatrix ma(reinterpret_cast<SbMat*>(T.data()));
            // mm -> m
            ma[3][0] *= 0.001f;
            ma[3][1] *= 0.001f;
            ma[3][2] *= 0.001f;
			m->matrix.setValue(ma);
		}
	}

	performCoMIK();
}
*/

