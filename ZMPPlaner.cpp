#include "ZMPPlaner.h"
#include <Inventor/nodes/SoSphere.h>

ZMPPlaner::ZMPPlaner() : _pPlaner(0), _bComputed(false), _nSamplesDS(0), _nSamplesSS(0)
{
	/* working
	_visualization = new SoSeparator();
	_referenceNodes = new SoSeparator();

	_swReference = new SoSwitch();
	_swRealZMP = new SoSwitch();

	//_visualization->addChild(_swReference);
	_visualization->addChild(_referenceNodes);
	_visualization->addChild(_swRealZMP);

	//_swReference->addChild(_referenceNodes);
	_referenceNodes->addChild(new SoSphere());
	_swReference->whichChild=SO_SWITCH_ALL;
	*/

	_visualization = new SoSeparator();
	_swReference = new SoSwitch();
	_swRealZMP = new SoSwitch();
	_referenceNodes = new SoSeparator();
	_realNodes = new SoSeparator();
	_swReference->whichChild=SO_SWITCH_ALL;
	_swRealZMP->whichChild=SO_SWITCH_ALL;

	_visualization->addChild(_swReference);
	_visualization->addChild(_swRealZMP);

	_swReference->addChild(_referenceNodes);
	//_referenceNodes->addChild(new SoSphere());
	

	/*
	_swReference->whichChild=SO_SWITCH_ALL;
	_swRealZMP->whichChild=SO_SWITCH_ALL;

	_referenceNodes = new SoSeparator();
	_realNodes = new SoSeparator();
	
	SoSeparator* sep1 = new SoSeparator();
	SoSeparator* sep2 = new SoSeparator();

	_visualization->addChild(sep1);
	sep1->addChild(_swReference);
	
	_visualization->addChild(sep2);
	sep2->addChild(_swRealZMP);


	_swReference->addChild(new SoSphere());

	_swRealZMP->addChild(_realNodes);
	_swReference->addChild(_referenceNodes);
*/
}

ZMPPlaner::~ZMPPlaner()
{
}

void ZMPPlaner::showReference(bool isVisible) 
{
	_swReference->whichChild=(isVisible?SO_SWITCH_ALL:SO_SWITCH_NONE);
	if (isVisible) 
		std::cout << "Make Reference ZMP visible!" << std::endl;
	else
		std::cout << "Make Reference ZMP invisible!" << std::endl;
}

void ZMPPlaner::showRealZMP(bool isVisible)
{
	_swRealZMP->whichChild=(isVisible?SO_SWITCH_ALL:SO_SWITCH_NONE);
	if (isVisible) 
		std::cout << "Make computed ZMP visible!" << std::endl;
}

// saves computed ZMP-Positions to given Matrix
// returns 1 if successful, 0 otherwise
int ZMPPlaner::getZMPPositions(Eigen::Matrix2Xf& zmp)
{
	if (_bComputed) 
	{
		zmp = _mZMP;
		return 1;
	}
	else
		return 0;
}