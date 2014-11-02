/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __DAMPENING_CONTROLLER_H__
#define __DAMPENING_CONTROLLER_H__

namespace Bipedal
{

struct DampeningController
{
    double D;
    double T;
    double delta;
    double lastError;

    void update(double error)
    {
        delta += 1.0/D * error - 1.0/T * delta;
        lastError = error;
    }
};

}

#endif

