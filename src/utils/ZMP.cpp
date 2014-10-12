#include <Eigen/Dense>

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Nodes/ContactSensor.h>

#include "bipedal.h"
#include "utils/ZMP.h"


namespace Bipedal
{

inline Eigen::Vector2f computeModelZMP(const Eigen::Vector3f& com, const Eigen::Vector3f& comAcc, double gravity)
{
    Eigen::Vector2f zmp;
    zmp.x() = com.x() - com.z() / gravity * comAcc.x();
    zmp.y() = com.y() - com.z() / gravity * comAcc.y();
    return zmp;
}

inline Eigen::Vector2f computeMultiBodyZMP(double mass,
                                           double gravity,
                                           const Eigen::Vector3f& com,
                                           const Eigen::Vector3f& linearMomentumDiff,
                                           const Eigen::Vector3f& angularMomentumDiff)
{
    Eigen::Vector2f zmp;
    double norm = mass * gravity + linearMomentumDiff.z();
    zmp.x() = mass * gravity * com.x() - angularMomentumDiff.y();
    zmp.y() = mass * gravity * com.y() + angularMomentumDiff.x();
    zmp /= norm;

    return zmp;
}

MultiBodyZMPEstimator::MultiBodyZMPEstimator(double mass, double gravity)
: mass(mass)
, gravity(gravity)
, estimation(Eigen::Vector2f::Zero())
, linearMomentumDiff(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero())
, angularMomentumDiff(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero())
{
    BOOST_ASSERT(gravity > 0);
}

void MultiBodyZMPEstimator::update(const Eigen::Vector3f& com,
            const Eigen::Vector3f& linearMomentum,
            const Eigen::Vector3f& angularMomentum,
            double dt)
{
    linearMomentumDiff.update(linearMomentum, dt);
    angularMomentumDiff.update(angularMomentum, dt);

    estimation = computeMultiBodyZMP(mass, gravity, com, linearMomentumDiff.estimation, angularMomentumDiff.estimation);
}

CartTableZMPEstimator::CartTableZMPEstimator(double gravity)
: gravity(gravity)
, accelerationEstimator(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero())
{
    BOOST_ASSERT(gravity > 0);
}

void CartTableZMPEstimator::update(const Eigen::Vector3f& com, const Eigen::Vector3f& comVel, double dt)
{
    accelerationEstimator.update(comVel, dt);
    estimation = computeModelZMP(com, accelerationEstimator.estimation, gravity);
}

CoPZMPEstimator::CoPZMPEstimator(const VirtualRobot::ContactSensorPtr& leftFootSensor,
                                 const VirtualRobot::ContactSensorPtr& rightFootSensor)
: leftFootSensor(leftFootSensor)
, rightFootSensor(rightFootSensor)
, estimation(Eigen::Vector2f::Zero())
{
}

void CoPZMPEstimator::update(float dt)
{
    double totalForce = 0.0;
    Eigen::Vector2f pointSum = Eigen::Vector2f::Zero();

    for (const auto& f : leftFootSensor->getContacts().forces)
    {
        if (f.bodyName == "Floor" && f.zForce > 0)
        {
            totalForce += f.zForce;
            pointSum += f.zForce * f.contactPoint.head(2);
        }
    }

    for (const auto& f : rightFootSensor->getContacts().forces)
    {
        if (f.bodyName == "Floor" && f.zForce > 0)
        {
            totalForce += f.zForce;
            pointSum += f.zForce * f.contactPoint.head(2);
        }
    }

    if (totalForce > 0)
    {
        estimation = pointSum / totalForce / 1000.0;
    }
}

}
