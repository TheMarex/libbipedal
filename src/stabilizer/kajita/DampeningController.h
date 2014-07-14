#ifndef __DAMPENING_CONTROLLER_H__
#define __DAMPENING_CONTROLLER_H__

struct DampeningController
{
    double D;
    double T;
    double delta;

    void update(double error)
    {
        delta += 1.0/D * error - 1.0/T * delta;
    }
};

#endif

