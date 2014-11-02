/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __KAJITA_STABILIZER_H__
#define __KAJITA_STABILIZER_H__

#include <SimDynamics/SimDynamics.h>
#include <SimDynamics/DynamicsEngine/DynamicsRobot.h>
#include <VirtualRobot/VirtualRobot.h>

#include <boost/shared_ptr.hpp>
#include <string>

#include "../bipedal.h"
#include "../utils/Estimation.h"

#include "kajita/ZMPDistributor.h"

#include "FrameAdaptingStabilizer.h"
#include "TorqueControllingStabilizer.h"

namespace Bipedal
{

class FootForceController;
class FootTorqueController;
class ZMPTrackingController;

typedef boost::shared_ptr<FootForceController> FootForceControllerPtr;
typedef boost::shared_ptr<FootTorqueController> FootTorqueControllerPtr;
typedef boost::shared_ptr<ZMPTrackingController> ZMPTrackingControllerPtr;

class KajitaStabilizer : public FrameAdaptingStabilizer, public TorqueControllingStabilizer
{
public:
    KajitaStabilizer(const VirtualRobot::RobotPtr& robot,
                     const VirtualRobot::RobotNodePtr& chest,
                     const VirtualRobot::RobotNodePtr& leftFoot,
                     const VirtualRobot::RobotNodePtr& rightFoot,
                     const VirtualRobot::RobotNodePtr& leftFootBody,
                     const VirtualRobot::RobotNodePtr& rightFootBody,
                     const VirtualRobot::RobotNodePtr& leftAnkleBody,
                     const VirtualRobot::RobotNodePtr& rightAnkleBody,
                     const VirtualRobot::RobotNodePtr& pelvis,
                     const VirtualRobot::ForceTorqueSensorPtr& leftAnkleSensorX,
                     const VirtualRobot::ForceTorqueSensorPtr& rightAnkleSensorX,
                     const VirtualRobot::ForceTorqueSensorPtr& leftAnkleSensorY,
                     const VirtualRobot::ForceTorqueSensorPtr& rightAnkleSensorY);

    //! in global frame
    virtual const Eigen::Matrix4f& getChestPoseRef() override { return chestPoseRef; }
    //! in global frame
    virtual const Eigen::Matrix4f& getPelvisPoseRef() override { return pelvisPoseRef; }
    //! in global frame
    virtual const Eigen::Matrix4f& getLeftFootPoseRef() override { return leftFootPoseRef; }
    //! in global frame
    virtual const Eigen::Matrix4f& getRightFootPoseRef() override { return rightFootPoseRef; }
    //! in ground frame
    virtual const Eigen::Vector3f& getCoMPositionRef() override { return comPositionRef; }
    //! in ground frame
    virtual const Eigen::Vector3f& getZMPPositionRef() override { return zmpPositionRef; }

    //! in global frame
    virtual const Eigen::Matrix4f& getChestPose() override { return chestPose; }
    //! in global frame
    virtual const Eigen::Matrix4f& getPelvisPose() override { return pelvisPose; }
    //! in global frame
    virtual const Eigen::Matrix4f& getLeftFootPose() override { return leftFootPose; }
    //! in global frame
    virtual const Eigen::Matrix4f& getRightFootPose() override { return rightFootPose; }
    //! in ground frame
    virtual const Eigen::Vector3f& getCoMPosition() override { return comPosition; }
    //! in ground frame
    virtual const Eigen::Vector3f& getZMPPosition() override { return zmpPosition; }

    virtual std::unordered_map<std::string, DampeningController*> getControllers();

    virtual const Eigen::Vector3f& getLeftAnkleTorque() override { return ft.leftTorque;}
    virtual const Eigen::Vector3f& getRightAnkleTorque() override { return ft.rightTorque;}
    virtual const Eigen::Vector3f& getLeftAnkleForce() override { return ft.leftForce;}
    virtual const Eigen::Vector3f& getRightAnkleForce() override { return ft.rightForce;}

    virtual void update(float dt,
                        const Eigen::Vector3f& comPosition,
                        const Eigen::Vector3f& comVelocity,
                        const Eigen::Vector3f& zmpPosition,
                        Bipedal::SupportPhase phase,
                        const Eigen::Matrix4f& chestPoseRef,
                        const Eigen::Matrix4f& pelvisPoseRef,
                        const Eigen::Matrix4f& leftFootPoseRef,
                        const Eigen::Matrix4f& rightFootPoseRef,
                        const Eigen::Vector3f& comPositionRef,
                        const Eigen::Vector3f& comVelocityRef,
                        const Eigen::Vector3f& zmpPositionRef) override;

private:
    void adaptFrame(Eigen::Matrix4f& frame);

    // needs to be affected by torso rotation + orientation
    VirtualRobot::RobotNodePtr chest;
    // should not be affected by torso rotation + orientation, only changes in leg angles
    VirtualRobot::RobotNodePtr pelvis;
    // joint + body of left foot
    VirtualRobot::RobotNodePtr leftFootBody;
    // joint + body of right foot
    VirtualRobot::RobotNodePtr rightFootBody;
    // TCP on left foot
    VirtualRobot::RobotNodePtr leftFoot;
    // TCP on right foot
    VirtualRobot::RobotNodePtr rightFoot;

    ZMPTrackingControllerPtr          zmpTrackingController;
    FootForceControllerPtr             footForceController;
    FootTorqueControllerPtr            footTorqueController;
    TwoDOFPostureControllerPtr         chestPostureController;
    ZMPDistributorPtr                forceDistributor;
    VirtualRobot::ForceTorqueSensorPtr leftAnkleSensorX;
    VirtualRobot::ForceTorqueSensorPtr rightAnkleSensorX;
    VirtualRobot::ForceTorqueSensorPtr leftAnkleSensorY;
    VirtualRobot::ForceTorqueSensorPtr rightAnkleSensorY;

    Eigen::Matrix4f chestPoseRef;
    Eigen::Matrix4f pelvisPoseRef;
    Eigen::Matrix4f leftFootPoseRef;
    Eigen::Matrix4f rightFootPoseRef;
    Eigen::Vector3f comPositionRef;
    Eigen::Vector3f zmpPositionRef;

    Eigen::Matrix4f chestPose;
    Eigen::Matrix4f pelvisPose;
    Eigen::Matrix4f leftFootPose;
    Eigen::Matrix4f rightFootPose;
    Eigen::Vector3f comPosition;
    Eigen::Vector3f zmpPosition;

    Eigen::Vector3f leftAnkleOffset;
    Eigen::Vector3f rightAnkleOffset;

    ZMPDistributor::ForceTorque ft;
};

}

#endif
