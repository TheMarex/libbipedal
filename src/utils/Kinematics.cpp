#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/IK/HierarchicalIK.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/IK/CoMIK.h>
#include <Eigen/Dense>

#include "utils/Kinematics.h"

namespace Kinematics {

void extractControlFrames(VirtualRobot::RobotPtr robot,
                          const Eigen::Matrix3Xf& leftFootTrajectory,
                          const Eigen::MatrixXf&  bodyTrajectory,
                          VirtualRobot::RobotNodeSetPtr bodyJoints,
                          VirtualRobot::RobotNodePtr node,
                          std::vector<Eigen::Matrix4f>& controlFrames)
{
    Eigen::Matrix4f leftInitialPose = bodyJoints->getKinematicRoot()->getGlobalPose();
    int N = leftFootTrajectory.cols();

    for (int i = 0; i < N; i++)
    {
        // Move basis along with the left foot
        Eigen::Matrix4f leftFootPose = leftInitialPose;
        leftFootPose.block(0, 3, 3, 1) = 1000 * leftFootTrajectory.col(i);
        robot->setGlobalPose(leftFootPose);
        bodyJoints->setJointValues(bodyTrajectory.col(i));

        controlFrames.push_back(node->getGlobalPose());
    }
}

void transformTrajectoryToGroundFrame(VirtualRobot::RobotPtr robot,
                                                  const Eigen::Matrix3Xf& leftFootTrajectory,
                                                  VirtualRobot::RobotNodePtr leftFoot,
                                                  VirtualRobot::RobotNodePtr rightFoot,
                                                  VirtualRobot::RobotNodeSetPtr bodyJoints,
                                                  const Eigen::MatrixXf& bodyTrajectory,
                                                  const Eigen::Matrix3Xf& trajectory,
                                                  const std::vector<Kinematics::SupportPhase>& phase,
                                      Eigen::Matrix3Xf& relativeTrajectory)
{
    Eigen::Matrix4f leftInitialPose = bodyJoints->getKinematicRoot()->getGlobalPose();
    int N = trajectory.cols();
    relativeTrajectory.resize(3, N);

    for (int i = 0; i < N; i++)
    {
        // Move basis along with the left foot
        Eigen::Matrix4f leftFootPose = leftInitialPose;
        leftFootPose.block(0, 3, 3, 1) = 1000 * leftFootTrajectory.col(i);
        robot->setGlobalPose(leftFootPose);
        bodyJoints->setJointValues(bodyTrajectory.col(i));
        Eigen::Matrix4f worldToRef = computeGroundFrame(leftFoot->getGlobalPose(), rightFoot->getGlobalPose(), phase[i]);
        Eigen::Vector4f homVec;
        homVec(3, 0) = 1;
        homVec.block(0, 0, 3, 1) = trajectory.col(i) * 1000;
        relativeTrajectory.col(i) = worldToRef.colPivHouseholderQr().solve(homVec).block(0, 0, 3, 1) / 1000.0;
    }
}

inline void computeStepConfiguration(VirtualRobot::RobotNodeSetPtr nodeSetJoints,
                                                 VirtualRobot::RobotNodeSetPtr nodeSetBodies,
                                                 VirtualRobot::RobotNodePtr waist,
                                                 const Eigen::Vector3f& targetCoM,
                                                 const Eigen::Vector3f& targetRightFoot,
                                                 const Eigen::Matrix4f& initialRightFootPose,
                                                 Eigen::VectorXf& result)
{
    const float ikPrec = 0.01;

    std::vector<VirtualRobot::HierarchicalIK::JacobiDefinition> jacobiDefinitions;

    // TCP constraint for right foot pose
    VirtualRobot::DifferentialIKPtr diffIK;
    diffIK.reset(new VirtualRobot::DifferentialIK(nodeSetJoints));
    Eigen::Matrix4f footGoal = initialRightFootPose;
    footGoal.block(0, 3, 3, 1) = targetRightFoot;
    diffIK->setGoal(footGoal);
    VirtualRobot::HierarchicalIK::JacobiDefinition jRightFoot;
    jRightFoot.jacProvider = diffIK;
    jacobiDefinitions.push_back(jRightFoot);

    // CoM constraint
    VirtualRobot::CoMIKPtr comIK;
    comIK.reset(new VirtualRobot::CoMIK(nodeSetJoints, nodeSetBodies));
    comIK->setGoal(targetCoM);
    VirtualRobot::HierarchicalIK::JacobiDefinition jCoM;
    jCoM.jacProvider = comIK;
    jacobiDefinitions.push_back(jCoM);

    // TCP constraint for upright upper body
    VirtualRobot::DifferentialIKPtr uprightBody;
    uprightBody.reset(new VirtualRobot::DifferentialIK(nodeSetJoints));
    Eigen::Matrix4f defaultOrientation = waist->getGlobalPose();
    uprightBody->setGoal(defaultOrientation, waist, VirtualRobot::IKSolver::Orientation);
    VirtualRobot::HierarchicalIK::JacobiDefinition jUprightBody;
    jUprightBody.jacProvider = uprightBody;
    jacobiDefinitions.push_back(jUprightBody);

    VirtualRobot::HierarchicalIK hIK(nodeSetJoints);

    float lastErrorLength = 1000.0f;

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

        Eigen::VectorXf delta = hIK.computeStep(jacobiDefinitions);
        Eigen::VectorXf jv;
        nodeSetJoints->getJointValues(jv);
        jv += delta;
        nodeSetJoints->setJointValues(jv);
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

    nodeSetJoints->getJointValues(result);
}

void computeWalkingTrajectory(const VirtualRobot::RobotPtr& robot,
                                          const VirtualRobot::RobotNodeSetPtr& nodeSet,
                                          const VirtualRobot::RobotNodeSetPtr& colModelNodeSet,
                                          const VirtualRobot::RobotNodePtr& waist,
                                          const Eigen::Matrix3Xf& comTrajectory,
                                          const Eigen::Matrix3Xf& rightFootTrajectory,
                                          const Eigen::Matrix3Xf& leftFootTrajectory,
                                          Eigen::MatrixXf& trajectory)
{
    int rows = nodeSet->getSize();

    trajectory.resize(rows, rightFootTrajectory.cols());

    Eigen::Matrix4f rightInitialPose = nodeSet->getTCP()->getGlobalPose();
    Eigen::Matrix4f leftInitialPose = nodeSet->getKinematicRoot()->getGlobalPose();

    Eigen::Vector3f com = colModelNodeSet->getCoM();

    Eigen::VectorXf configuration;
    int N = trajectory.cols();

    for (int i = 0; i < N; i++)
    {
        // Move basis along with the left foot
        Eigen::Matrix4f leftFootPose = leftInitialPose;
        leftFootPose.block(0, 3, 3, 1) = 1000 * leftFootTrajectory.col(i);
        robot->setGlobalPose(leftFootPose);

        std::cout << "Frame #" << i << ", ";
        computeStepConfiguration(nodeSet,
                                 colModelNodeSet,
                                 waist,
                                 1000 * comTrajectory.col(i),
                                 1000 * rightFootTrajectory.col(i),
                                 rightInitialPose,
                                 configuration);

        trajectory.col(i) = configuration;
    }
}

}
