/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __HIERARCHICAL_REFERENCE_IK_H__
#define __HIERARCHICAL_REFERENCE_IK_H__

#include "ReferenceIK.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/IK/HierarchicalIK.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/IK/CoMIK.h>
#include <Eigen/Dense>

#include "utils/Kinematics.h"

namespace Bipedal
{

class HierarchicalReferenceIK : public ReferenceIK
{
public:
    HierarchicalReferenceIK(const VirtualRobot::RobotNodeSetPtr& nodes,
                            const VirtualRobot::RobotPtr& robot,
                            const VirtualRobot::RobotNodeSetPtr& colModel,
                            const VirtualRobot::RobotNodePtr& leftFootTCP,
                            const VirtualRobot::RobotNodePtr& rightFootTCP,
                            const VirtualRobot::RobotNodePtr& chest,
                            const VirtualRobot::RobotNodePtr& pelvis)
    : nodes(nodes)
    , robot(robot)
    , leftFootTCP(leftFootTCP)
    , rightFootTCP(rightFootTCP)
    , chest(chest)
    , pelvis(pelvis)
    , rightFootIK(new VirtualRobot::DifferentialIK(nodes))
    , chestIK(new VirtualRobot::DifferentialIK(nodes))
    , pelvisIK(new VirtualRobot::DifferentialIK(nodes))
    , comIK(new VirtualRobot::CoMIK(nodes, colModel))
    , hIK(nodes)
    {
        VirtualRobot::HierarchicalIK::JacobiDefinition jRightFoot;
        jRightFoot.jacProvider = rightFootIK;
        VirtualRobot::HierarchicalIK::JacobiDefinition jChest;
        jChest.jacProvider = chestIK;
        VirtualRobot::HierarchicalIK::JacobiDefinition jPelvis;
        jPelvis.jacProvider = pelvisIK;
        VirtualRobot::HierarchicalIK::JacobiDefinition jCoM;
        jCoM.jacProvider = comIK;
        jacobiDefinitions.push_back(jRightFoot);
        jacobiDefinitions.push_back(jCoM);
        jacobiDefinitions.push_back(jPelvis);
        jacobiDefinitions.push_back(jChest);
    }

    virtual const VirtualRobot::RobotNodeSetPtr& getNodes() override { return nodes; }

    virtual bool computeStep(const Eigen::Matrix4f& leftFootPose,
                             const Eigen::Matrix4f& rightFootPose,
                             const Eigen::Matrix4f& chestPose,
                             const Eigen::Matrix4f& pelvisPose,
                             const Eigen::Vector3f& comPosition,
                             Bipedal::SupportPhase phase,
                             Eigen::VectorXf &result) override
    {
        const float ikPrec = 0.1;

        /*
         * If the left foot is not the support foot we have to adapt
         * the target poses, since the left foot will not keep static.
         */
        if (phase & Bipedal::SUPPORT_RIGHT)
        {
            Eigen::Matrix4f leftSwingAdaption = leftFootTCP->getGlobalPose() * leftFootPose.inverse();
            rightFootIK->setGoal((Eigen::Matrix4f) (leftSwingAdaption * rightFootPose), rightFootTCP);
            chestIK->setGoal((Eigen::Matrix4f) (leftSwingAdaption * chestPose),  chest, VirtualRobot::IKSolver::Orientation);
            pelvisIK->setGoal((Eigen::Matrix4f) (leftSwingAdaption * pelvisPose), pelvis);
            comIK->setGoal((Eigen::Vector3f) VirtualRobot::MathTools::transformPosition(comPosition, leftSwingAdaption));
        }
        else
        {
            rightFootIK->setGoal(rightFootPose, rightFootTCP);
            chestIK->setGoal(chestPose,  chest, VirtualRobot::IKSolver::Orientation);
            pelvisIK->setGoal(pelvisPose, pelvis);
            comIK->setGoal(comPosition);
        }

        // before we start the IK backup the current angles, since we will modify the model
        Eigen::VectorXf origAngles;
        nodes->getJointValues(origAngles);

        VirtualRobot::HierarchicalIK hIK(nodes);

        result.resize(nodes->getSize(), 1);

        float lastErrorLength = std::numeric_limits<float>::max();
        float bestError = std::numeric_limits<float>::max();
        unsigned i = 0;
        Eigen::VectorXf angles;
        nodes->getJointValues(angles);
        Eigen::VectorXf bestAngles;
        for(; i < 100; i++)
        {
            float e = 0;
            for(int j = 0; j < jacobiDefinitions.size(); j++)
            {
                e += jacobiDefinitions[j].jacProvider->getError().norm();
            }

            if(e < bestError)
            {
                bestError = e;
                bestAngles = angles;
            }

            if(e > 10*lastErrorLength)
            {
                break;
            }
            else if(e < ikPrec)
            {
                break;
            }

            lastErrorLength = e;

            Eigen::VectorXf delta = hIK.computeStep(jacobiDefinitions);
            nodes->getJointValues(angles);
            angles += delta;
            nodes->setJointValues(angles);
        }

        result = bestAngles;
        nodes->setJointValues(bestAngles);

        bool correct = true;
        for(int j = 0; j < jacobiDefinitions.size(); j++)
        {
            if(jacobiDefinitions[j].jacProvider->getError().norm() > ikPrec)
            {
                correct = false;
                break;
            }
        }

        if (!correct)
        {
            std::cout << "IK failed: Iteratoins " << i
                      << " RightFoot: " << jacobiDefinitions[0].jacProvider->getError().norm()
                      << " CoM: " << jacobiDefinitions[1].jacProvider->getError().norm()
                      << " Pelvis: " << jacobiDefinitions[2].jacProvider->getError().norm()
                      << " Chest: " << jacobiDefinitions[3].jacProvider->getError().norm() << std::endl;
        }

        double angle;
        const double angleMax = 5.0 / 180 * M_PI;
        if (phase & Bipedal::SUPPORT_LEFT)
        {
            angle = zAxisAngle(leftFootTCP->getGlobalPose());
            if (angle > angleMax)
            {
                std::cout << "Warning: Left foot has no floor contact: delta " << angle / M_PI * 180.0 << "°." << std::endl;
                //correct = false;
            }
        }
        if (phase & Bipedal::SUPPORT_RIGHT)
        {
            angle = zAxisAngle(rightFootTCP->getGlobalPose());
            if (angle > angleMax)
            {
                std::cout << "Warning: Right foot has no floor contact: delta " << angle / M_PI * 180.0 << "°." << std::endl;
                //correct = false;
            }
        }

        // restore joint angles to not break the simox model
        nodes->setJointValues(origAngles);

        return correct;
    }

    // returns angle between zAxis and zAxsis in base coordinate system
    double zAxisAngle(const Eigen::Matrix4f& pose)
    {
        Eigen::Vector3f zAxis = pose.block(0, 2, 3, 1);
        double angle = acos(zAxis[2] / zAxis.norm());
        return angle;
    }

private:

    std::vector<VirtualRobot::HierarchicalIK::JacobiDefinition> jacobiDefinitions;
    VirtualRobot::RobotNodeSetPtr nodes;
    VirtualRobot::RobotPtr robot;
    VirtualRobot::RobotNodePtr leftFootTCP;
    VirtualRobot::RobotNodePtr rightFootTCP;
    VirtualRobot::RobotNodePtr chest;
    VirtualRobot::RobotNodePtr pelvis;
    VirtualRobot::DifferentialIKPtr leftFootIK;
    VirtualRobot::DifferentialIKPtr rightFootIK;
    VirtualRobot::DifferentialIKPtr pelvisIK;
    VirtualRobot::DifferentialIKPtr chestIK;
    VirtualRobot::CoMIKPtr comIK;
    VirtualRobot::HierarchicalIK hIK;
};

}

#endif

