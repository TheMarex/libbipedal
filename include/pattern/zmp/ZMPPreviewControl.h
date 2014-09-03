#ifndef _ZMP_PREVIEW_CONTROL_
#define _ZMP_PREVIEW_CONTROL_

#include "ZMPPlaner.h"

class ZMPPreviewControl : public ZMPPlaner
{
public:
    ZMPPreviewControl(const FootstepPlanerPtr& footstepPlaner,
                      const ZMPReferencePlanerPtr& refPlaner,
                      double comHeight=0.86);
    ~ZMPPreviewControl();

    void setPreview(int N);

protected:
    virtual void computeCoMTrajectory() override;

    int _nPreviewCount;
};

#endif

