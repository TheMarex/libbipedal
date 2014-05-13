#include "ZMPVisualization.h"

#include "VisualizationUtils.h"
#include <Inventor/nodes/SoSphere.h>

ZMPVisualization::ZMPVisualization(ZMPPlanerPtr planer)
: planer(planer)
{
	_visualization = new SoSeparator();
	_swReference = new SoSwitch();
	_swRealZMP = new SoSwitch();
    _swCoM = new SoSwitch();
    _referenceNodes = new SoSeparator();
	_realNodes = new SoSeparator();
    _comNodes = new SoSeparator();
    _swReference->whichChild=SO_SWITCH_ALL;
	_swRealZMP->whichChild=SO_SWITCH_ALL;
    _swCoM->whichChild=SO_SWITCH_ALL;

	_visualization->addChild(_swReference);
	_visualization->addChild(_swRealZMP);
    _visualization->addChild(_swCoM);

	_swReference->addChild(_referenceNodes);
    _swRealZMP->addChild(_realNodes);
    _swCoM->addChild(_comNodes);
}

void ZMPVisualization::showReference(bool isVisible) 
{
	_swReference->whichChild=(isVisible?SO_SWITCH_ALL:SO_SWITCH_NONE);
	if (isVisible) 
		std::cout << "Make Reference ZMP visible!" << std::endl;
	else
		std::cout << "Make Reference ZMP invisible!" << std::endl;
}

void ZMPVisualization::showRealZMP(bool isVisible)
{
	_swRealZMP->whichChild=(isVisible?SO_SWITCH_ALL:SO_SWITCH_NONE);
	if (isVisible) 
		std::cout << "Make computed ZMP visible!" << std::endl;
	else
		std::cout << "Make computed ZMP invisible!" << std::endl;
}

void ZMPVisualization::showCoM(bool isVisible)
{
	_swCoM->whichChild=(isVisible?SO_SWITCH_ALL:SO_SWITCH_NONE);
	if (isVisible) 
		std::cout << "Make CoM Trajectory visible!" << std::endl;
	else
		std::cout << "Make CoM Trajectory invisible!" << std::endl;
}

void ZMPVisualization::buildReferenceZMPVisualization() 
{
	_referenceNodes->removeAllChildren();
	SoMaterial* colorMat = new SoMaterial();
	colorMat->ambientColor.setValue(0.8,0.1,0.1);
	colorMat->diffuseColor.setValue(0.8,0.1,0.1);
	_referenceNodes->addChild(colorMat);

    // ZMP-reference is only available in a 2X Matrix, but we need 3X Matrix for visualization => put the 2D in the 3D Matrix
    const Eigen::Matrix2Xf& refZMP = planer->getReferenceZMPTrajectory();
	Eigen::Matrix3Xf positions;
	positions.resize(3, refZMP.cols());
	positions.setZero();
	positions.block(0,0,2, refZMP.cols()) = refZMP;
	std::cout << "Number of entries in position matrix: " << std::endl << positions.cols() << std::endl;

    VisualizationUtils::generateVisualizationForLineTrajectories(_referenceNodes, positions);
}

// Visualize the real-ZMP computed from CoM trajectory ...  TODO: check again
void ZMPVisualization::buildComputedZMPVisualization()
{
    _realNodes->removeAllChildren();

    const Eigen::Matrix2Xf& computedZMP = planer->getComputedZMPTrajectory();
    Eigen::Matrix3Xf positions;
    positions.resize(3, computedZMP.cols());
    positions.setZero();
    positions.block(0,0,2,computedZMP.cols()) = computedZMP;
    std::cout << "Number of entries in realZMP-position matrix: " << std::endl << positions.cols() << std::endl;

    Eigen::Matrix4f mFrom, mTo;
    mFrom.setIdentity();
    mTo.setIdentity();
    VisualizationUtils::generateVisualizationForLineTrajectories(_realNodes,positions, 0.1f, 0.8f, 0.1f);
}

// Visualize the CoM trajectory computed from reference ZMP - TODO: check for error
void ZMPVisualization::buildCoMVisualization()
{
    _comNodes->removeAllChildren();

    SoMaterial* colorMat = new SoMaterial();
    colorMat->ambientColor.setValue(0.1,0.1,0.8);
    colorMat->diffuseColor.setValue(0.1,0.1,0.8);
    _comNodes->addChild(colorMat);

    SoSeparator* sep = new SoSeparator();
    SoSphere* sp = new SoSphere();
    sp->radius = 0.01;
    sep->addChild(sp);

    VisualizationUtils::generateVisualizationDuplicatesFromTrajectories(_comNodes, sep, planer->getCoMTrajectory()); // (Eigen::Matrix3Xf) positions.block(0,0,3,100));
}

