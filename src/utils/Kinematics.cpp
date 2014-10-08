#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/Nodes/ContactSensor.h>
#include <VirtualRobot/IK/HierarchicalIK.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/IK/CoMIK.h>
#include <Eigen/Dense>

#include "utils/Kinematics.h"

namespace Bipedal
{

void extractControlFrames(VirtualRobot::RobotPtr robot,
                          const Eigen::Matrix3Xf& leftFootTrajectory,
                          const Eigen::MatrixXf&  bodyTrajectory,
                          VirtualRobot::RobotNodeSetPtr bodyJoints,
                          VirtualRobot::RobotNodePtr node,
                          std::vector<Eigen::Matrix4f>& controlFrames)
{
    Eigen::Matrix4f leftInitialPose = bodyJoints->getKinematicRoot()->getGlobalPose();
    int N = leftFootTrajectory.cols();

    for (int i = 0; i < N; i++)
    {
        // Move basis along with the left foot
        Eigen::Matrix4f leftFootPose = leftInitialPose;
        leftFootPose.block(0, 3, 3, 1) = 1000 * leftFootTrajectory.col(i);
        robot->setGlobalPose(leftFootPose);
        bodyJoints->setJointValues(bodyTrajectory.col(i));

        controlFrames.push_back(node->getGlobalPose());
    }
}



SupportPhaseSensor::SupportPhaseSensor(const VirtualRobot::ContactSensorPtr& leftFootSensor,
                                       const VirtualRobot::ContactSensorPtr& rightFootSensor)
: sameStateThreshold(0.01)
, sameStateTime(0)
, phase(SUPPORT_NONE)
, nextPhase(SUPPORT_NONE)
, initialized(false)
, leftFootSensor(leftFootSensor)
, rightFootSensor(rightFootSensor)
{
}

SupportPhase SupportPhaseSensor::getContactPhase() const
{
    auto floorComp = [](const VirtualRobot::ContactSensor::ContactForce& f)
                     {
                         return f.bodyName == "Floor";
                     };
    bool leftContact = std::any_of(leftFootSensor->getContacts().forces.begin(),
                                   leftFootSensor->getContacts().forces.end(),
                                    floorComp);
    bool rightContact = std::any_of(rightFootSensor->getContacts().forces.begin(),
                                    rightFootSensor->getContacts().forces.end(),
                                    floorComp);
    if (leftContact && rightContact)
        return SUPPORT_BOTH;
    if (leftContact)
        return SUPPORT_LEFT;
    if (rightContact)
        return SUPPORT_RIGHT;
    return SUPPORT_NONE;
}

void SupportPhaseSensor::update(float dt)
{
    SupportPhase sensorPhase = getContactPhase();

    if (sensorPhase == nextPhase)
    {
        sameStateTime += dt;
    }
    else
    {
        if (sameStateTime > 0)
        {
            sameStateTime -= dt;
        }
        else
        {
            sameStateTime = 0;
            nextPhase = sensorPhase;
        }
    }

    if (sameStateTime >= sameStateThreshold)
    {
        phase = nextPhase;
        sameStateTime = 0;
        initialized = true;
    }

    // before the phase has setteled just return the unfiltered values
    if (!initialized)
    {
        phase = sensorPhase;
    }
}

}
