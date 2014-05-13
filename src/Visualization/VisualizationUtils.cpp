#include "VisualizationUtils.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>

#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/actions/SoWriteAction.h>

namespace VisualizationUtils {

/*
 * This method can generate a trajectory of a certain object in a given SoSeparator-Node
 * The Object is inserted according to the translation values given in the Matrix3d
 */
void generateVisualizationDuplicatesFromTrajectories(SoSeparator* whereToInsert, SoSeparator* whatToInsert, const Eigen::Matrix3Xf &whereToTranslate) {
	Eigen::Vector3f pos, prev;
	pos.setZero();
	prev.setZero();
	for (int i=0; i < whereToTranslate.cols(); i++)
	{
		prev = pos;
		pos = whereToTranslate.col(i);
		// if consecutive positions are the same, display the item just once
		if ((i!=0) && (prev==pos))
			continue;
		SoSeparator *pSep = new SoSeparator();
		SoTranslation *pTranslate = new SoTranslation();
		pTranslate->translation.setValue(pos.x(), pos.y(), pos.z());
		whereToInsert->addChild(pSep);
		pSep->addChild(pTranslate);
		// TODO: add a node for transparency
		pSep->addChild(whatToInsert);
		//whatToInsert->ref();
	}
}

void generateVisualizationForLineTrajectories(SoSeparator* whereToInsert, const Eigen::Matrix3Xf& positionList, float rColor, float gColor, float bColor, float scale)
{
    Eigen::Matrix4f mFrom, mTo;
    mFrom.setIdentity();
    mTo.setIdentity();
    for (int i=0; i<positionList.cols()-1; i++) {
        mFrom.block(0,3,3,1) = positionList.col(i)*scale;
        mTo.block(0,3,3,1) = positionList.col(i+1)*scale;
        //VirtualRobot::VisualizationNodePtr p = visualizationFactory->createLine(vFrom, vTo, 2.0f, 0.1f, 0.8f, 0.1f);
        //_realNodes->addChild(VirtualRobot::CoinVisualizationFactory::getCoinVisualization(p));
        whereToInsert->addChild(VirtualRobot::CoinVisualizationFactory::createCoinLine(mFrom, mTo, 5.0f, rColor, gColor, bColor));
    }

}

void generateVisualizationForLineTrajectories(SoSeparator* whereToInsert, const Eigen::Vector3f& from, const Eigen::Vector3f& to, float rColor, float gColor, float bColor,  float scale)
{
    Eigen::Matrix4f mFrom, mTo;
    mFrom.setIdentity();
    mTo.setIdentity();
    mFrom.block(0,3,3,1) = from*scale;
    mTo.block(0,3,3,1) = to*scale;
    whereToInsert->addChild(VirtualRobot::CoinVisualizationFactory::createCoinLine(mFrom, mTo, 5.0f, rColor, gColor, bColor));
}

/*
 * Save as scene Graph to a test file
 */
void writeSceneGraphToFile(SoSeparator* node)
{
	SoOutput* output = new SoOutput();
	output->openFile("sceneGraphDebug.txt");
	SoWriteAction* pWrite = new SoWriteAction(output);
	pWrite->apply(node);
	//node->write(pWrite);
	std::cout << "Scene Graph saved to file!" << std::endl;
}

}
