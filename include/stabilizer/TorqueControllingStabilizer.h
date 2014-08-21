#ifndef __TORQUE_CONTROLLING_STABILIZER_H__
#define __TORQUE_CONTROLLING_STABILIZER_H__

#include <Eigen/Dense>

class TorqueControllingStabilizer
{
public:
    virtual const Eigen::Vector3f& getLeftAnkleTorque() = 0;
    virtual const Eigen::Vector3f& getRightAnkleTorque() = 0;
    virtual const Eigen::Vector3f& getLeftAnkleForce() = 0;
    virtual const Eigen::Vector3f& getRightAnkleForce() = 0;
};

#endif
