#ifndef _ZMP_PREVIEW_CONTROL_
#define _ZMP_PREVIEW_CONTROL_

#include "pattern/zmp/ZMPPlaner.h"

class ZMPPreviewControl : public ZMPPlaner
{
public:
    ZMPPreviewControl();
    ~ZMPPreviewControl();

    virtual void computeReference();
    void setPreview(int N);
    void computeCoM();

protected:
    int _nPreviewCount;
};

#endif

