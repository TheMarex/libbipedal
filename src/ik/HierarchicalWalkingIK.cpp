#include <VirtualRobot/Robot.h>
#include <VirtualRobot/IK/HierarchicalIK.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/IK/CoMIK.h>

#include "bipedal/ik/HierarchicalWalkingIK.h"
#include "bipedal/utils/Kinematics.h"

HierarchicalWalkingIK::HierarchicalWalkingIK(const VirtualRobot::RobotPtr& robot,
                                             const VirtualRobot::RobotNodeSetPtr& nodeSet,
                                             const VirtualRobot::RobotNodeSetPtr& colModelNodeSet,
                                             const VirtualRobot::RobotNodePtr& chest,
                                             const VirtualRobot::RobotNodePtr& pelvis,
                                             const VirtualRobot::RobotNodePtr& leftFootTCP,
                                             const VirtualRobot::RobotNodePtr& rightFootTCP)
: WalkingIK(robot, nodeSet, colModelNodeSet, chest, pelvis, leftFootTCP, rightFootTCP)
, rightFootIK(new VirtualRobot::DifferentialIK(nodeSet))
, uprightBodyIK(new VirtualRobot::DifferentialIK(nodeSet))
, straightPelvisIK(new VirtualRobot::DifferentialIK(nodeSet))
, comIK(new VirtualRobot::CoMIK(nodeSet, colModelNodeSet))
, hIK(new VirtualRobot::HierarchicalIK(nodeSet))
{
    VirtualRobot::HierarchicalIK::JacobiDefinition jRightFoot;
    jRightFoot.jacProvider = rightFootIK;
    jacobiDefinitions.push_back(jRightFoot);

    VirtualRobot::HierarchicalIK::JacobiDefinition jCoM;
    jCoM.jacProvider = comIK;
    jacobiDefinitions.push_back(jCoM);

    VirtualRobot::HierarchicalIK::JacobiDefinition jStraightPelvis;
    jStraightPelvis.jacProvider = straightPelvisIK;
    jacobiDefinitions.push_back(jStraightPelvis);

    VirtualRobot::HierarchicalIK::JacobiDefinition jUprightBody;
    jUprightBody.jacProvider = uprightBodyIK;
    jacobiDefinitions.push_back(jUprightBody);
}

void HierarchicalWalkingIK::computeWalkingTrajectory(const Eigen::Matrix3Xf& comTrajectory,
                                                     const Eigen::Matrix3Xf& rightFootTrajectory,
                                                     const Eigen::Matrix3Xf& leftFootTrajectory,
                                                     std::vector<Eigen::Matrix3f>& rootOrientation,
                                                     Eigen::MatrixXf& trajectory)
{
    int rows = nodeSet->getSize();

    trajectory.resize(rows, rightFootTrajectory.cols());
    rootOrientation.resize(rightFootTrajectory.cols());

    Eigen::Matrix4f rightInitialPose = rightFootTCP->getGlobalPose();
    Eigen::Matrix4f leftInitialPose  = leftFootTCP->getGlobalPose();
    rightInitialPose.block(0,0,3,3) = Eigen::Matrix3f::Identity();
    leftInitialPose.block(0,0,3,3)  = Eigen::Matrix3f::Identity();
    Eigen::Matrix4f chestInitialPose = chest->getGlobalPose();
    Eigen::Matrix4f pelvisInitialPose = pelvis->getGlobalPose();

    Eigen::Vector3f com = colModelNodeSet->getCoM();

    Eigen::VectorXf configuration;
    int N = trajectory.cols();

    Eigen::Vector3f yAxisLeft  = leftInitialPose.block(0, 1, 3, 1);
    Eigen::Vector3f yAxisRight = rightInitialPose.block(0, 1, 3, 1);
    Eigen::Matrix4f leftFootPose  = leftInitialPose;
    Eigen::Matrix4f rightFootPose = rightInitialPose;
    Eigen::Matrix4f chestPose = chestInitialPose;
    Eigen::Matrix4f pelvisPose = pelvisInitialPose;

    for (int i = 0; i < N; i++)
    {
        leftFootPose.block(0, 3, 3, 1) = 1000 * leftFootTrajectory.col(i);
        leftFootPose.block(0, 0, 3, 3) = Kinematics::poseFromYAxis(yAxisLeft);
        robot->setGlobalPose(leftFootPose);

        rightFootPose.block(0, 3, 3, 1) = 1000 * rightFootTrajectory.col(i);
        rightFootPose.block(0, 0, 3, 3) = Kinematics::poseFromYAxis(yAxisRight);

        // FIXME the orientation of the chest and chest is specific to armar 4
        // since the x-Axsis points in walking direction
        Eigen::Vector3f xAxisChest = (yAxisLeft + yAxisRight)/2;
        xAxisChest.normalize();
        chestPose.block(0, 0, 3, 3) = Kinematics::poseFromXAxis(xAxisChest);
        pelvisPose.block(0, 0, 3, 3) = Kinematics::poseFromYAxis(-xAxisChest);

        // compute pose for next step:
        // Use vector between old position and new as y-Axsis to realize
        // a changing walking direction.
        if (i < N-1)
        {
            Eigen::Vector2f leftDirection = leftFootTrajectory.col(i+1).block(0, 0, 2, 1) - leftFootTrajectory.col(i).block(0, 0, 2, 1);
            if (leftDirection.norm() > 0.00001)
            {
                yAxisLeft.block(0, 0, 2, 1) = leftDirection / leftDirection.norm();
                yAxisLeft.normalize();
            }
            Eigen::Vector2f rightDirection = rightFootTrajectory.col(i+1).block(0, 0, 2, 1) - rightFootTrajectory.col(i).block(0, 0, 2, 1);
            if (rightDirection.norm() > 0.00001)
            {
                yAxisRight.block(0, 0, 2, 1) = rightDirection / rightDirection.norm();
                yAxisRight.normalize();
            }
        }

        std::cout << "Frame #" << i << ", ";
        computeStepConfiguration(1000 * comTrajectory.col(i),
                                 rightFootPose,
                                 chestPose,
                                 pelvisPose,
                                 configuration);

        trajectory.col(i) = configuration;
        rootOrientation[i] = leftFootPose.block(0, 0, 3, 3);
    }
}

void HierarchicalWalkingIK::computeStepConfiguration(const Eigen::Vector3f& targetCoM,
                                                     const Eigen::Matrix4f& targetRightFootPose,
                                                     const Eigen::Matrix4f& targetChestPose,
                                                     const Eigen::Matrix4f& targetPelvisPose,
                                                     Eigen::VectorXf& result)
{
    const float ikPrec = 0.5;

    rightFootIK->setGoal(targetRightFootPose);
    comIK->setGoal(targetCoM);
    uprightBodyIK->setGoal(targetChestPose, chest, VirtualRobot::IKSolver::Orientation);
    straightPelvisIK->setGoal(targetPelvisPose, pelvis, VirtualRobot::IKSolver::Orientation);

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

