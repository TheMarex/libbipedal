/*

Copyright (c) 2014, Ömer Terlemez, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef _ZMP_PREVIEW_CONTROL_
#define _ZMP_PREVIEW_CONTROL_

#include "ZMPPlaner.h"

namespace Bipedal
{

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

}

#endif

