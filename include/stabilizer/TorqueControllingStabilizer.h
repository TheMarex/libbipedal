/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __TORQUE_CONTROLLING_STABILIZER_H__
#define __TORQUE_CONTROLLING_STABILIZER_H__

#include <Eigen/Dense>

namespace Bipedal
{

class TorqueControllingStabilizer
{
public:
    virtual const Eigen::Vector3f& getLeftAnkleTorque() = 0;
    virtual const Eigen::Vector3f& getRightAnkleTorque() = 0;
    virtual const Eigen::Vector3f& getLeftAnkleForce() = 0;
    virtual const Eigen::Vector3f& getRightAnkleForce() = 0;
};

}

#endif
