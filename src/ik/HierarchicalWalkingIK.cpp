#include <VirtualRobot/Robot.h>
#include <VirtualRobot/IK/HierarchicalIK.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/IK/CoMIK.h>

#include "bipedal/ik/HierarchicalWalkingIK.h"
#include "bipedal/utils/Kinematics.h"

HierarchicalWalkingIK::HierarchicalWalkingIK(const VirtualRobot::RobotPtr& robot,
                                             const VirtualRobot::RobotNodeSetPtr& outputNodeSet,
                                             const VirtualRobot::RobotNodeSetPtr& ikNodeSet,
                                             const VirtualRobot::RobotNodeSetPtr& colModelNodeSet,
                                             const VirtualRobot::RobotNodePtr& chest,
                                             const VirtualRobot::RobotNodePtr& pelvis,
                                             const VirtualRobot::RobotNodePtr& leftFootTCP,
                                             const VirtualRobot::RobotNodePtr& rightFootTCP)
: WalkingIK(robot, outputNodeSet, ikNodeSet, colModelNodeSet, chest, pelvis, leftFootTCP, rightFootTCP)
, rightFootIK(new VirtualRobot::DifferentialIK(ikNodeSet))
, uprightBodyIK(new VirtualRobot::DifferentialIK(ikNodeSet))
, straightPelvisIK(new VirtualRobot::DifferentialIK(ikNodeSet))
, comIK(new VirtualRobot::CoMIK(ikNodeSet, colModelNodeSet))
, hIK(new VirtualRobot::HierarchicalIK(ikNodeSet))
{
    VirtualRobot::HierarchicalIK::JacobiDefinition jUprightBody;
    jUprightBody.jacProvider = uprightBodyIK;
    jacobiDefinitions.push_back(jUprightBody);

    VirtualRobot::HierarchicalIK::JacobiDefinition jStraightPelvis;
    jStraightPelvis.jacProvider = straightPelvisIK;
    jacobiDefinitions.push_back(jStraightPelvis);

    VirtualRobot::HierarchicalIK::JacobiDefinition jCoM;
    jCoM.jacProvider = comIK;
    jacobiDefinitions.push_back(jCoM);

    VirtualRobot::HierarchicalIK::JacobiDefinition jRightFoot;
    jRightFoot.jacProvider = rightFootIK;
    jacobiDefinitions.push_back(jRightFoot);
}

void HierarchicalWalkingIK::computeWalkingTrajectory(const Eigen::Matrix3Xf& comTrajectory,
                                                     const Eigen::Matrix6Xf& rightFootTrajectory,
                                                     const Eigen::Matrix6Xf& leftFootTrajectory,
                                                     std::vector<Eigen::Matrix3f>& rootOrientation,
                                                     Eigen::MatrixXf& trajectory)
{
    int rows = outputNodeSet->getSize();

    trajectory.resize(rows, rightFootTrajectory.cols());
    rootOrientation.resize(rightFootTrajectory.cols());

    Eigen::Vector3f com = colModelNodeSet->getCoM();

    Eigen::VectorXf configuration;
    int N = trajectory.cols();

    Eigen::Matrix4f leftFootPose  = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f rightFootPose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f chestPose     = chest->getGlobalPose();
    Eigen::Matrix4f pelvisPose    = pelvis->getGlobalPose();

    for (int i = 0; i < N; i++)
    {
        VirtualRobot::MathTools::posrpy2eigen4f(1000 * leftFootTrajectory.block(0, i, 3, 1),  leftFootTrajectory.block(3, i, 3, 1),  leftFootPose);
        VirtualRobot::MathTools::posrpy2eigen4f(1000 * rightFootTrajectory.block(0, i, 3, 1), rightFootTrajectory.block(3, i, 3, 1), rightFootPose);

        // FIXME the orientation of the chest and chest is specific to armar 4
        // since the x-Axsis points in walking direction
        Eigen::Vector3f xAxisChest = (leftFootPose.block(0, 1, 3, 1) + rightFootPose.block(0, 1, 3, 1))/2;
        xAxisChest.normalize();
        chestPose.block(0, 0, 3, 3) = Kinematics::poseFromXAxis(xAxisChest);
        pelvisPose.block(0, 0, 3, 3) = Kinematics::poseFromYAxis(-xAxisChest);

        std::cout << "Frame #" << i << ", ";
        robot->setGlobalPose(leftFootPose);
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
        ikNodeSet->getJointValues(jv);
        jv += delta;
        ikNodeSet->setJointValues(jv);
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

    std::cout << (ok ? "IK Ok" : "IK Failed") << std::endl;

    outputNodeSet->getJointValues(result);
}

