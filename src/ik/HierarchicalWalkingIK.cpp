#include <VirtualRobot/Robot.h>
#include <VirtualRobot/IK/HierarchicalIK.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/IK/CoMIK.h>

#include "bipedal/ik/HierarchicalWalkingIK.h"

HierarchicalWalkingIK::HierarchicalWalkingIK(const VirtualRobot::RobotPtr& robot,
                                             const VirtualRobot::RobotNodeSetPtr& nodeSet,
                                             const VirtualRobot::RobotNodeSetPtr& colModelNodeSet,
                                             const VirtualRobot::RobotNodePtr& waist,
                                             const VirtualRobot::RobotNodePtr& leftFootTCP,
                                             const VirtualRobot::RobotNodePtr& rightFootTCP)
: WalkingIK(robot, nodeSet, colModelNodeSet, waist, leftFootTCP, rightFootTCP)
, rightFootIK(new VirtualRobot::DifferentialIK(nodeSet))
, uprightBodyIK(new VirtualRobot::DifferentialIK(nodeSet))
, comIK(new VirtualRobot::CoMIK(nodeSet, colModelNodeSet))
, hIK(new VirtualRobot::HierarchicalIK(nodeSet))
{
    VirtualRobot::HierarchicalIK::JacobiDefinition jRightFoot;
    jRightFoot.jacProvider = rightFootIK;
    jacobiDefinitions.push_back(jRightFoot);

    VirtualRobot::HierarchicalIK::JacobiDefinition jCoM;
    jCoM.jacProvider = comIK;
    jacobiDefinitions.push_back(jCoM);

    VirtualRobot::HierarchicalIK::JacobiDefinition jUprightBody;
    jUprightBody.jacProvider = uprightBodyIK;
    jacobiDefinitions.push_back(jUprightBody);
}

void HierarchicalWalkingIK::computeWalkingTrajectory(const Eigen::Matrix3Xf& comTrajectory,
                                      const Eigen::Matrix3Xf& rightFootTrajectory,
                                      const Eigen::Matrix3Xf& leftFootTrajectory,
                                      Eigen::MatrixXf& trajectory)
{
    int rows = nodeSet->getSize();

    trajectory.resize(rows, rightFootTrajectory.cols());

    Eigen::Matrix4f rightInitialPose = rightFootTCP->getGlobalPose();
    Eigen::Matrix4f leftInitialPose  = leftFootTCP->getGlobalPose();
    Eigen::Matrix4f waistInitialPose = waist->getGlobalPose();

    Eigen::Vector3f com = colModelNodeSet->getCoM();

    Eigen::VectorXf configuration;
    int N = trajectory.cols();

    for (int i = 0; i < N; i++)
    {
        // Move basis along with the left foot
        Eigen::Matrix4f leftFootPose = leftInitialPose;
        leftFootPose.block(0, 3, 3, 1) = 1000 * leftFootTrajectory.col(i);
        robot->setGlobalPose(leftFootPose);

        Eigen::Matrix4f rightFootPose = rightInitialPose;
        rightFootPose.block(0, 3, 3, 1) = 1000 * rightFootTrajectory.col(i);

        std::cout << "Frame #" << i << ", ";
        computeStepConfiguration(1000 * comTrajectory.col(i),
                                 rightFootPose,
                                 waistInitialPose,
                                 configuration);

        trajectory.col(i) = configuration;
    }
}

void HierarchicalWalkingIK::computeStepConfiguration(const Eigen::Vector3f& targetCoM,
                                                     const Eigen::Matrix4f& targetRightFootPose,
                                                     const Eigen::Matrix4f& targetWaistPose,
                                                     Eigen::VectorXf& result)
{
    const float ikPrec = 0.01;

    rightFootIK->setGoal(targetRightFootPose);
    comIK->setGoal(targetCoM);
    uprightBodyIK->setGoal(targetWaistPose, waist, VirtualRobot::IKSolver::Orientation);

    float lastErrorLength = std::numeric_limits<float>::max();
    for (int i = 0; i < 50; i++)
    {
        float e = 0;

        for (int j = 0; j < jacobiDefinitions.size(); j++)
        {
            e += jacobiDefinitions[j].jacProvider->getError().norm();
        }

        if (e > 2 * lastErrorLength)
        {
            break;
        }
        else if (e < ikPrec)
        {
            break;
        }

        lastErrorLength = e;

        Eigen::VectorXf delta = hIK->computeStep(jacobiDefinitions);
        Eigen::VectorXf jv;
        nodeSet->getJointValues(jv);
        jv += delta;
        nodeSet->setJointValues(jv);
    }

    float e1 = jacobiDefinitions[0].jacProvider->getError().norm();
    float e2 = jacobiDefinitions[1].jacProvider->getError().norm();

    bool ok = true;

    for (int i = 0; i < jacobiDefinitions.size(); i++)
    {
        float e = jacobiDefinitions[i].jacProvider->getError().norm();

        std::cout << "e_" << i << "=" << e << "   ";

        if (e > ikPrec)
        {
            ok = false;
        }
    }

    std::cout << ((e1 <= ikPrec && e2 <= ikPrec) ? "IK Ok" : "IK Failed") << std::endl;

    nodeSet->getJointValues(result);
}

