#ifndef __VISU_UTILS_H__
#define __VISU_UTILS_H__

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSwitch.h>
#include <Inventor/nodes/SoMaterial.h>

#include <Eigen/Dense>

namespace VisualizationUtils
{
	void generateVisualizationDuplicatesFromTrajectories(SoSeparator* whereToInsert,
        SoSeparator* whatToInsert,
        const Eigen::Matrix3Xf& whereToTranslate);
    void generateVisualizationForLineTrajectories(SoSeparator* whereToInsert,
        const Eigen::Vector3f& from,
        const Eigen::Vector3f& to,
        float rColor = 0.8f,
        float gColor = 0.1f,
        float bColor = 0.1f,
        float scale = 1000.0f);
    void generateVisualizationForLineTrajectories(SoSeparator* whereToInsert,
        const Eigen::Matrix3Xf& positionList,
        float rColor = 0.8f,
        float gColor = 0.1f,
        float bColor = 0.1f,
        float scale = 1000.0f);
    void writeSceneGraphToFile(SoSeparator* node);
}

#endif
