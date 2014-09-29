#ifndef __DIFFERENTIAL_REFERENCE_IK_H__
#define __DIFFERENTIAL_REFERENCE_IK_H__

#include "ReferenceIK.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotFactory.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/IK/HierarchicalIK.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <Eigen/Dense>

#include "../utils/Kinematics.h"

namespace Bipedal
{

class DifferentialReferenceIK : public ReferenceIK
{
    struct IKNodes
    {
        VirtualRobot::RobotNodeSetPtr supportToPelvisNodes;
        VirtualRobot::RobotNodeSetPtr pelvisToSwingNodes;
        std::vector<VirtualRobot::RobotNodePtr> leftToRightNodes;

        VirtualRobot::RobotNodePtr pelvis;
        VirtualRobot::RobotNodePtr swingFoot;

        VirtualRobot::RobotPtr robot;
    };

public:
    DifferentialReferenceIK(const VirtualRobot::RobotNodeSetPtr& nodes,
                            const VirtualRobot::RobotPtr& robot,
                            const VirtualRobot::RobotNodeSetPtr& colModel,
                            const VirtualRobot::RobotNodePtr& leftFootTCP,
                            const VirtualRobot::RobotNodePtr& rightFootTCP,
                            const VirtualRobot::RobotNodePtr& chest,
                            const VirtualRobot::RobotNodePtr& pelvis)
    : robot(robot)
    // TODO Simox needs ::cloneReversed to clone + reverse all nodes
    , robotReversed(VirtualRobot::RobotFactory::cloneInversed(robot, rightFootTCP->getName()))
    , nodes(nodes)
    , chest(chest)
    , torsoNodes(VirtualRobot::RobotNodeSet::createRobotNodeSet(robot, "TrosoJoints",
                                                std::vector<std::string> {
                                                    "Torso_Joint1",
                                                    "Torso_Joint2"
                                                }))
    {
        BOOST_ASSERT(robotReversed);
        BOOST_ASSERT(robot);

        VirtualRobot::RobotNodeSetPtr leftToRight = robot->getRobotNodeSet("Left2RightLeg");
        VirtualRobot::RobotNodeSetPtr rightToLeft = constructReverseSet("RighToLeftLeg", robotReversed, leftToRight);

        leftSupportIKNodes.robot = robot;
        leftSupportIKNodes.leftToRightNodes = leftToRight->getAllRobotNodes();
        // FIXME don't hardcore joint names
        leftSupportIKNodes.supportToPelvisNodes = getNodeSubset(leftToRight, "LeftLeg_Joint6", "LeftLeg_Joint1");
        leftSupportIKNodes.pelvisToSwingNodes = getNodeSubset(leftToRight, "RightLeg_Joint1", "RightLeg_Joint6");
        leftSupportIKNodes.pelvis = pelvis;
        leftSupportIKNodes.swingFoot = rightFootTCP;

        // get other nodes and correct the order
        std::vector<VirtualRobot::RobotNodePtr> otherLeftToRight = rightToLeft->getAllRobotNodes();
        std::reverse(otherLeftToRight.begin(), otherLeftToRight.end());

        rightSupportIKNodes.robot = robotReversed;
        rightSupportIKNodes.leftToRightNodes = otherLeftToRight;
        rightSupportIKNodes.supportToPelvisNodes = getNodeSubset(rightToLeft, "RightLeg_Joint6", "RightLeg_Joint1");
        rightSupportIKNodes.pelvisToSwingNodes = getNodeSubset(rightToLeft, "LeftLeg_Joint1", "LeftLeg_Joint6");
        rightSupportIKNodes.pelvis = robotReversed->getRobotNode(pelvis->getName());
        rightSupportIKNodes.swingFoot = robotReversed->getRobotNode(leftFootTCP->getName());
    }

    VirtualRobot::RobotNodeSetPtr getNodeSubset(VirtualRobot::RobotNodeSetPtr set, const std::string& startName, const std::string& endName)
    {
        std::vector<VirtualRobot::RobotNodePtr> subsetNodes;
        bool foundStart = false;
        bool foundEnd = false;
        for (int i = 0; i < set->getSize(); i++)
        {
            if (!foundStart && (*set)[i]->getName() == startName)
            {
                foundStart = true;
            }
            if (foundStart)
            {
                subsetNodes.push_back((*set)[i]);
            }
            if ((*set)[i]->getName() == endName)
            {
                foundEnd = true;
                break;
            }
        }

        BOOST_ASSERT(foundStart);
        BOOST_ASSERT(foundEnd);

        return VirtualRobot::RobotNodeSet::createRobotNodeSet(set->getRobot(), startName + "_" + endName, subsetNodes);
    }

    // TODO move to RobotFactory
    VirtualRobot::RobotNodeSetPtr constructReverseSet(const std::string& name,
                                                      const VirtualRobot::RobotPtr& reversedRobot,
                                                      const VirtualRobot::RobotNodeSetPtr& originalSet)
    {
        std::vector<std::string> nodeNames;
        for (auto& n : *originalSet)
        {
            nodeNames.push_back(n->getName());
        }

        // we need to reverse this, so the RobotNodeSet will pick the correct TCP/root
        std::reverse(nodeNames.begin(), nodeNames.end());

        return VirtualRobot::RobotNodeSet::createRobotNodeSet(reversedRobot, name, nodeNames);
    }

    VirtualRobot::RobotPtr getInvertedRobot() { return robotReversed; }

    bool computeFootIK(const Eigen::Matrix4f& supportRootPose,
                       const Eigen::Matrix4f& supportTCPPose,
                       const Eigen::Matrix4f& swingTCPPose,
                       const IKNodes& ikNodes)
    {
        BOOST_ASSERT(ikNodes.robot);
        BOOST_ASSERT(ikNodes.supportToPelvisNodes);
        BOOST_ASSERT(ikNodes.pelvisToSwingNodes);
        BOOST_ASSERT(ikNodes.pelvis);
        BOOST_ASSERT(ikNodes.swingFoot);

        // we assume that the support foot is the root node
        ikNodes.robot->setGlobalPose(supportRootPose);

        VirtualRobot::DifferentialIK supportToPelvisIK(ikNodes.supportToPelvisNodes);
        VirtualRobot::DifferentialIK pelvisToSwingIK(ikNodes.pelvisToSwingNodes);

        const float ikPrec = 1.0;
        const float minChange = 0;
        const unsigned numSteps = 100;

        supportToPelvisIK.setGoal(supportTCPPose, ikNodes.pelvis, VirtualRobot::IKSolver::All, ikPrec);
        pelvisToSwingIK.setGoal(swingTCPPose, ikNodes.swingFoot, VirtualRobot::IKSolver::All, ikPrec);

        BOOST_ASSERT(supportToPelvisIK.getJacobianMatrix().rows() > 0);
        BOOST_ASSERT(pelvisToSwingIK.getJacobianMatrix().rows() > 0);

        bool success = true;
        success = success && supportToPelvisIK.computeSteps(ikPrec, minChange, numSteps);
        success = success && pelvisToSwingIK.computeSteps(ikPrec, minChange, numSteps);

        std::cout << "Errors: Pelvis: " << supportToPelvisIK.getError().norm() << " Swing foot: " << pelvisToSwingIK.getError().norm() << std::endl;

        return success;
    }

    bool computeChestIK(const Eigen::Matrix4f& chestPose)
    {
        VirtualRobot::DifferentialIK chestIK(torsoNodes);

        const float ikPrec = 1.0;
        const float minChange = 0;
        const unsigned numSteps = 100;

        chestIK.setGoal(chestPose, chest, VirtualRobot::IKSolver::Orientation, ikPrec, 1.0 / 180.0 * M_PI);

        bool success = chestIK.computeSteps(ikPrec, minChange, numSteps);

        std::cout << "Errors: Chest: " << chestIK.getError().norm() << std::endl;

        return success;
    }

    void extractAngles(const std::vector<VirtualRobot::RobotNodePtr>& nodes, Eigen::VectorXf& result)
    {
        result.resize(nodes.size());
        for (unsigned i = 0; i < nodes.size(); i++)
        {
            result[i] = nodes[i]->getJointValue();
        }
    }

    void syncRobotModels(const VirtualRobot::RobotPtr& source, const VirtualRobot::RobotPtr& target)
    {
        if (source == target)
            return;

        VirtualRobot::RobotNodePtr root = target->getRootNode();
        target->setGlobalPose(source->getRobotNode(root->getName())->getGlobalPose());

        //FIXME this is not really fast
        for (auto& rn : source->getRobotNodes())
        {
            auto targetNode = target->getRobotNode(rn->getName());
            targetNode->setJointValue(rn->getJointValue());

            // Make sure we didn't hit a joint limit
            BOOST_ASSERT(targetNode->getJointValue() == rn->getJointValue());
        }
    }

    virtual bool computeStep(const Eigen::Matrix4f& leftFootPose,
                             const Eigen::Matrix4f& rightFootPose,
                             const Eigen::Matrix4f& chestPose,
                             const Eigen::Matrix4f& pelvisPose,
                             const Eigen::Vector3f& comPosition,
                             Bipedal::SupportPhase phase,
                             Eigen::VectorXf &result) override
    {
        syncRobotModels(robot, robotReversed);

        Eigen::Matrix4f rootPose;
        Eigen::VectorXf legsResult;
        switch (phase)
        {
            case Bipedal::SUPPORT_LEFT:
                computeFootIK(leftFootPose, pelvisPose, rightFootPose, leftSupportIKNodes);
                extractAngles(leftSupportIKNodes.leftToRightNodes, legsResult);
                rootPose = leftSupportIKNodes.robot->getRobotNode(robot->getRootNode()->getName())->getGlobalPose();
                break;
            case Bipedal::SUPPORT_RIGHT:
                computeFootIK(rightFootPose, pelvisPose, leftFootPose, rightSupportIKNodes);
                extractAngles(rightSupportIKNodes.leftToRightNodes, legsResult);
                rootPose = rightSupportIKNodes.robot->getRobotNode(robot->getRootNode()->getName())->getGlobalPose();
                break;
            // as default use left support, even in dual support phase
            default:
                computeFootIK(leftFootPose, pelvisPose, rightFootPose, leftSupportIKNodes);
                extractAngles(leftSupportIKNodes.leftToRightNodes, legsResult);
                rootPose = leftSupportIKNodes.robot->getRobotNode(robot->getRootNode()->getName())->getGlobalPose();
                break;
        }

        auto leftToRightNodes = robot->getRobotNodeSet("Left2RightLeg");

        // sync values with original robot
        for (int i = 0; i < legsResult.size(); i++)
        {
            (*leftToRightNodes)[i]->setJointValue(legsResult[i]);
        }
        robot->setGlobalPose(rootPose);

        // now try to correct chest posture
        computeChestIK(chestPose);

        result.resize(nodes->getSize(), 1);
        nodes->getJointValues(result);

        return true;
    }

private:
    VirtualRobot::RobotPtr robot;
    VirtualRobot::RobotPtr robotReversed;
    VirtualRobot::RobotNodePtr chest;
    VirtualRobot::RobotNodeSetPtr torsoNodes;
    VirtualRobot::RobotNodeSetPtr nodes;
    IKNodes leftSupportIKNodes;
    IKNodes rightSupportIKNodes;
};

}

#endif

