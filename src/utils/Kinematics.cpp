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
                          const Eigen::Matrix6Xf& leftFootTrajectory,
                          const Eigen::MatrixXf&  bodyTrajectory,
                          VirtualRobot::RobotNodeSetPtr bodyJoints,
                          VirtualRobot::RobotNodePtr node,
                          std::vector<Eigen::Matrix4f>& controlFrames)
{
    int N = leftFootTrajectory.cols();

    for (int i = 0; i < N; i++)
    {
        // Move basis along with the left foot
        Eigen::Matrix4f leftFootPose = Eigen::Matrix4f::Identity();
        VirtualRobot::MathTools::posrpy2eigen4f(1000 * leftFootTrajectory.block(0, i, 3, 1),
                                                leftFootTrajectory.block(3, i, 3, 1),  leftFootPose);
        robot->setGlobalPose(leftFootPose);
        bodyJoints->setJointValues(bodyTrajectory.col(i));

        controlFrames.push_back(node->getGlobalPose());
    }
}

void transformOrientationToGroundFrame(const VirtualRobot::RobotPtr& robot,
                                       const Eigen::Matrix6Xf& leftFootTrajectory,
                                       const VirtualRobot::RobotNodePtr& leftFoot,
                                       const VirtualRobot::RobotNodePtr& rightFoot,
                                       const VirtualRobot::RobotNodeSetPtr& bodyJoints,
                                       const Eigen::MatrixXf& bodyTrajectory,
                                       const Eigen::Matrix3Xf& trajectory,
                                       const std::vector<SupportInterval>& intervals,
                                       Eigen::Matrix3Xf& relativeTrajectory)
{
    Eigen::Matrix4f leftInitialPose = bodyJoints->getKinematicRoot()->getGlobalPose();
    int N = trajectory.cols();
    int M = trajectory.rows();
    relativeTrajectory.resize(M, N);

    BOOST_ASSERT(M > 0 && M <= 3);

    auto intervalIter = intervals.begin();
    for (int i = 0; i < N; i++)
    {
        while (i >= intervalIter->endIdx)
        {
            intervalIter = std::next(intervalIter);
        }
        // Move basis along with the left foot
        Eigen::Matrix4f leftFootPose = Eigen::Matrix4f::Identity();
        VirtualRobot::MathTools::posrpy2eigen4f(1000 * leftFootTrajectory.block(0, i, 3, 1),
                                                leftFootTrajectory.block(3, i, 3, 1),  leftFootPose);
        robot->setGlobalPose(leftFootPose);
        bodyJoints->setJointValues(bodyTrajectory.col(i));
        Eigen::Matrix3f worldToRef = computeGroundFrame(leftFoot->getGlobalPose(),
                                                        rightFoot->getGlobalPose(),
                                                        intervalIter->phase).block(0, 0, 3, 3);
        relativeTrajectory.block(0, i, M, 1) = worldToRef.colPivHouseholderQr().solve(trajectory.col(i)).block(0, 0, M, 1);
    }
}

SupportPhaseSensor::SupportPhaseSensor(const VirtualRobot::ContactSensorPtr& leftFootSensor,
                                       const VirtualRobot::ContactSensorPtr& rightFootSensor)
: sameStateThreshold(0.05)
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
