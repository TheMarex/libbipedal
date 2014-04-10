#ifndef _ZMP_PREVIEW_CONTROL_
#define _ZMP_PREVIEW_CONTROL_

#include "ZMPPlaner.h"

//typedef boost::shared_ptr<ZMPPreviewControl> BOOST_ZMPCONTROLER;

class ZMPPreviewControl : public ZMPPlaner
{
public:
	ZMPPreviewControl();
	~ZMPPreviewControl();

	virtual void computeReference();
	void setPreview(int N);
	void computeCoM();

    void computeWalkingTrajectory();
    void computeStepConfiguration(const std::string &nodeSetName, const std::string &colModelName, const Eigen::Vector3f &targetCoM, const Eigen::Vector3f &targetFoot, const Eigen::Matrix4f &initialFootPose, Eigen::VectorXf &result);

    const Eigen::MatrixXf &getWalkingTrajectory() { return trajectory; }
    Eigen::Matrix3Xf getLeftFootTrajectory() { return _pPlaner->getLeftFootTrajectory(); }

protected:
	void buildReferenceVisualization();
    void buildRealZMPVisualization();
    void buildCoMVisualization();

protected:
	int _nPreviewCount;

    Eigen::MatrixXf trajectory;

};

#endif

