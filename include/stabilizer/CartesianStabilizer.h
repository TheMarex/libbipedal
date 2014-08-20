#ifndef __CARTESIAN_STABILIZER_H__
#define __CARTESIAN_STABILIZER_H__

#include <SimDynamics/SimDynamics.h>
#include <SimDynamics/DynamicsEngine/DynamicsRobot.h>
#include <VirtualRobot/VirtualRobot.h>

#include <boost/shared_ptr.hpp>
#include <string>

#include "../bipedal.h"

#include "FrameAdaptingStabilizer.h"

class TwoDOFPostureController;

typedef boost::shared_ptr<TwoDOFPostureController> TwoDOFPostureControllerPtr;

class CartesianStabilizer : public FrameAdaptingStabilizer
{
public:

    CartesianStabilizer(const VirtualRobot::RobotPtr& robot,
                        const VirtualRobot::RobotNodeSetPtr& nodes,
                        const VirtualRobot::RobotNodePtr& chest,
                        const VirtualRobot::RobotNodePtr& leftFoot,
                        const VirtualRobot::RobotNodePtr& rightFoot,
                        const VirtualRobot::RobotNodePtr& pelvis,
                        ReferenceIKPtr referenceIK);

    virtual const Eigen::Matrix4f& getChestPoseRef() override { return chestPoseRef; }
    virtual const Eigen::Matrix4f& getPelvisPoseRef() override { return pelvisPoseRef; }
    virtual const Eigen::Matrix4f& getLeftFootPoseRef() override { return leftFootPoseRef; }
    virtual const Eigen::Matrix4f& getRightFootPoseRef() override { return rightFootPoseRef; }
    virtual const Eigen::Vector3f& getCoMPositionRef() override { return comPositionRef; }
    virtual const Eigen::Vector3f& getZMPPositionRef() override { return zmpPositionRef; }

    virtual const Eigen::Matrix4f& getChestPose() override { return chestPose; }
    virtual const Eigen::Matrix4f& getPelvisPose() override { return pelvisPose; }
    virtual const Eigen::Matrix4f& getLeftFootPose() override { return leftFootPose; }
    virtual const Eigen::Matrix4f& getRightFootPose() override { return rightFootPose; }

    virtual const Eigen::VectorXf& getResultAngles() override { return resultAngles; }
    virtual const VirtualRobot::RobotNodeSetPtr& getNodes() override { return nodes; }

    virtual void update(float dt,
                Kinematics::SupportPhase phase,
                const Eigen::Vector3f& zmp,
                const Eigen::Matrix4f& chestPoseRef,
                const Eigen::Matrix4f& pelvisPoseRef,
                const Eigen::Matrix4f& leftFootPoseRef,
                const Eigen::Matrix4f& rightFootPoseRef,
                const Eigen::Vector3f& comPositionRef) override;

private:
    void adaptFrame(Eigen::Matrix4f& frame);

    // needs to be affected by torso rotation + orientation
    VirtualRobot::RobotNodePtr chest;
    // should not be affected by torso rotation + orientation, only changes in leg angles
    VirtualRobot::RobotNodePtr pelvis;
    // TCP on left foot
    VirtualRobot::RobotNodePtr leftFoot;
    // TCP on right foot
    VirtualRobot::RobotNodePtr rightFoot;
    // all nodes that are used for the IK
    VirtualRobot::RobotNodeSetPtr nodes;

    TwoDOFPostureControllerPtr chestPostureController;
    TwoDOFPostureControllerPtr leftFootPostureController;
    TwoDOFPostureControllerPtr rightFootPostureController;
    TwoDOFPostureControllerPtr pelvisPostureController;
    ReferenceIKPtr             referenceIK;

    std::vector<VirtualRobot::RobotNodePtr> trajectoryNodes;

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

    Eigen::Matrix4f stepAdaptionFrame;

    Eigen::Matrix4f rootPose;
    Eigen::VectorXf resultAngles;
};


#endif
