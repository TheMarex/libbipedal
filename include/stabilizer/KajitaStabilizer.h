#ifndef __KAJITA_STABILIZER_H__
#define __KAJITA_STABILIZER_H__

#include <SimDynamics/SimDynamics.h>
#include <SimDynamics/DynamicsEngine/DynamicsRobot.h>
#include <VirtualRobot/VirtualRobot.h>

#include <boost/shared_ptr.hpp>
#include <string>

#include "../bipedal.h"

#include "kajita/ForceDistributor.h"

class FootForceController;
class FootTorqueController;
class ChestPostureController;

typedef boost::shared_ptr<FootForceController> FootForceControllerPtr;
typedef boost::shared_ptr<FootTorqueController> FootTorqueControllerPtr;
typedef boost::shared_ptr<ChestPostureController> ChestPostureControllerPtr;

class KajitaStabilizer
{
public:


    KajitaStabilizer(const VirtualRobot::RobotPtr& robot,
                     const VirtualRobot::ForceTorqueSensorPtr& leftAnkleSensor,
                     const VirtualRobot::ForceTorqueSensorPtr& rightAnkleSensor);

    const Eigen::Matrix4f& getChestPoseRef() { return chestPoseRef; }
    const Eigen::Matrix4f& getPelvisPoseRef() { return pelvisPoseRef; }
    const Eigen::Matrix4f& getLeftFootPoseRef() { return leftFootPoseRef; }
    const Eigen::Matrix4f& getRightFootPoseRef() { return rightFootPoseRef; }

    const Eigen::Matrix4f& getChestPose() { return chestPose; }
    const Eigen::Matrix4f& getPelvisPose() { return pelvisPose; }
    const Eigen::Matrix4f& getLeftFootPose() { return leftFootPose; }
    const Eigen::Matrix4f& getRightFootPose() { return rightFootPose; }

    const Eigen::Vector3f& getLeftAnkleTorque() { return ft.leftTorque;}
    const Eigen::Vector3f& getRightAnkleTorque() { return ft.rightTorque;}
    const Eigen::Vector3f& getLeftAnkleForce() { return ft.leftForce;}
    const Eigen::Vector3f& getRightAnkleForce() { return ft.rightForce;}

    const Eigen::VectorXf& getResultAngles() { return resultAngles; }
    const VirtualRobot::RobotNodeSetPtr& getNodes() { return nodes; }

    void update(float dt,
                Kinematics::SupportPhase phase,
                const Eigen::Vector3f& zmp,
                const Eigen::Matrix4f& chestPoseRef,
                const Eigen::Matrix4f& pelvisPoseRef,
                const Eigen::Matrix4f& leftFootPoseRef,
                const Eigen::Matrix4f& rightFootPoseRef);

private:
    void adaptFrame(Eigen::Matrix4f& frame);

    // needs to be affected by torso rotation + orientation
    VirtualRobot::RobotNodePtr chest;
    // should not be affected by torso rotation + orientation, only changes in leg angles
    VirtualRobot::RobotNodePtr pelvis;
    // TCP on left foot
    VirtualRobot::RobotNodePtr leftAnkle;
    // TCP on right foot
    VirtualRobot::RobotNodePtr rightAnkle;
    // joint + body of left foot
    VirtualRobot::RobotNodePtr leftFoot;
    // joint + body of right foot
    VirtualRobot::RobotNodePtr rightFoot;
    // all nodes that are used for the IK
    VirtualRobot::RobotNodeSetPtr nodes;

    FootForceControllerPtr             footForceController;
    FootTorqueControllerPtr            footTorqueController;
    ChestPostureControllerPtr          chestPostureController;
    ForceDistributorPtr                forceDistributor;
    VirtualRobot::ForceTorqueSensorPtr leftAnkleSensor;
    VirtualRobot::ForceTorqueSensorPtr rightAnkleSensor;
    ReferenceIKPtr                     referenceIK;

    std::vector<VirtualRobot::RobotNodePtr> trajectoryNodes;

    Eigen::Matrix4f chestPoseRef;
    Eigen::Matrix4f pelvisPoseRef;
    Eigen::Matrix4f leftFootPoseRef;
    Eigen::Matrix4f rightFootPoseRef;

    Eigen::Matrix4f chestPose;
    Eigen::Matrix4f pelvisPose;
    Eigen::Matrix4f leftFootPose;
    Eigen::Matrix4f rightFootPose;

    Eigen::Matrix4f rootPose;
    Eigen::VectorXf resultAngles;
    ForceDistributor::ForceTorque ft;
};


#endif
