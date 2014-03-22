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
    void computeInverseKinematics();

protected:
	void buildReferenceVisualization();
    void buildRealZMPVisualization();
    void buildCoMVisualization();

protected:
	int _nPreviewCount;

};

#endif

