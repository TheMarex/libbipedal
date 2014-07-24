#ifndef __DIFFERENTIAL_REFERENCE_IK_H__
#define __DIFFERENTIAL_REFERENCE_IK_H__

#include "ReferenceIK.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/IK/HierarchicalIK.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <Eigen/Dense>

class DifferentialReferenceIK : public ReferenceIK
{
public:
    DifferentialReferenceIK(const VirtualRobot::RobotNodeSetPtr& nodes,
                            const VirtualRobot::RobotPtr& robot,
                            const VirtualRobot::RobotNodePtr& leftFootTCP,
                            const VirtualRobot::RobotNodePtr& rightFootTCP,
                            const VirtualRobot::RobotNodePtr& chest,
                            const VirtualRobot::RobotNodePtr& pelvis)
    : robot(robot)
    , nodes(nodes)
    , leftLegNodes(robot->getRobotNodeSet("LeftLeg"))
    , rightLegNodes(robot->getRobotNodeSet("RightLeg"))
    , pelvisNodes(robot->getRobotNodeSet("Robot"))
    , chestNodes(robot->getRobotNodeSet("Robot"))
    , leftFootTCP(leftFootTCP)
    , rightFootTCP(rightFootTCP)
    , chest(chest)
    , pelvis(pelvis)
    , leftFootIK(new VirtualRobot::DifferentialIK(leftLegNodes))
    , rightFootIK(new VirtualRobot::DifferentialIK(rightLegNodes))
    , chestIK(new VirtualRobot::DifferentialIK(chestNodes))
    , pelvisIK(new VirtualRobot::DifferentialIK(pelvisNodes))
    {
    }

    virtual bool computeStep(const Eigen::Matrix4f& leftFootPose,
                             const Eigen::Matrix4f& rightFootPose,
                             const Eigen::Matrix4f& chestPose,
                             const Eigen::Matrix4f& pelvisPose,
                             const Eigen::Vector3f& comPosition,
                             Kinematics::SupportPhase phase,
                             Eigen::VectorXf &result) override
    {
        const float ikPrec = 0.1;

        Eigen::Matrix4f leftLegAdaption = leftFootTCP->getGlobalPose() * leftFootPose.inverse();
        leftFootIK->setGoal((Eigen::Matrix4f) (leftLegAdaption * pelvisPose), pelvis);
        rightFootIK->setGoal(rightFootPose, rightFootTCP);
        chestIK->setGoal(chestPose,  chest, VirtualRobot::IKSolver::Orientation);
        //pelvisIK->setGoal(pelvisPose, pelvis, VirtualRobot::IKSolver::Orientation);

        const float minChange = 0;
        const unsigned numSteps = 100;

        bool success = true;
        success = success && leftFootIK->computeSteps(ikPrec, minChange, numSteps);
        success = success && rightFootIK->computeSteps(ikPrec, minChange, numSteps);
        success = success && chestIK->computeSteps(ikPrec, minChange, numSteps);
        if (!success)
        {
            std::cout << "IK failed: LeftFoot: " << leftFootIK->getError().norm()
                      << " RightFoot: " << rightFootIK->getError().norm()
                      << " Chest: " << chestIK->getError().norm() << std::endl;
        }

        result.resize(nodes->getSize(), 1);
        nodes->getJointValues(result);

        return true;
    }

private:

    std::vector<VirtualRobot::HierarchicalIK::JacobiDefinition> jacobiDefinitions;
    VirtualRobot::RobotPtr robot;
    VirtualRobot::RobotNodeSetPtr nodes;
    VirtualRobot::RobotNodeSetPtr leftLegNodes;
    VirtualRobot::RobotNodeSetPtr rightLegNodes;
    VirtualRobot::RobotNodeSetPtr pelvisNodes;
    VirtualRobot::RobotNodeSetPtr chestNodes;
    VirtualRobot::RobotNodePtr leftFootTCP;
    VirtualRobot::RobotNodePtr rightFootTCP;
    VirtualRobot::RobotNodePtr chest;
    VirtualRobot::RobotNodePtr pelvis;
    VirtualRobot::DifferentialIKPtr leftFootIK;
    VirtualRobot::DifferentialIKPtr rightFootIK;
    VirtualRobot::DifferentialIKPtr pelvisIK;
    VirtualRobot::DifferentialIKPtr chestIK;
};

#endif

