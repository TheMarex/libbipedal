#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/IK/HierarchicalIK.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/IK/CoMIK.h>
#include <Eigen/Dense>

namespace Kinematics {

enum SupportPhase
{
    SUPPORT_NONE  = 0,
    SUPPORT_LEFT  = 1,
    SUPPORT_RIGHT = 2,
    SUPPORT_BOTH  = 3
};

/**
 * Extract a vector of poses of the given node as the robot follows the given
 * angle trajectories.
 */
inline void extractControlFrames(VirtualRobot::RobotPtr robot,
                                const Eigen::Matrix3Xf& leftFootTrajectory,
                                const Eigen::MatrixXf&  bodyTrajectory,
                                VirtualRobot::RobotNodeSetPtr bodyJoints,
                                VirtualRobot::RobotNodePtr node,
                                std::vector<Eigen::Matrix4f>& controlFrames)
{
    Eigen::Matrix4f leftInitialPose = bodyJoints->getKinematicRoot()->getGlobalPose();
    int N = leftFootTrajectory.cols();
    for(int i = 0; i < N; i++)
    {
        // Move basis along with the left foot
        Eigen::Matrix4f leftFootPose = leftInitialPose;
        leftFootPose.block(0,3,3,1) = 1000 * leftFootTrajectory.col(i);
        robot->setGlobalPose(leftFootPose);
        bodyJoints->setJointValues(bodyTrajectory.col(i));

        controlFrames.push_back(node->getGlobalPose());
    }
}

/*
 * Return pose of ground frame.
 *
 * Warning: Since the frame comes form simox it is in mm!
 */
inline Eigen::Matrix4f computeGroundFrame(VirtualRobot::RobotNodePtr leftFoot,
                                    VirtualRobot::RobotNodePtr rightFoot,
                                    SupportPhase phase)
{
    Eigen::Matrix4f refToWorld = Eigen::Matrix4f::Identity();
    switch (phase)
    {
        case SUPPORT_LEFT:
            refToWorld = leftFoot->getGlobalPose();
            break;
        case SUPPORT_RIGHT:
            refToWorld = rightFoot->getGlobalPose();
            break;
        case SUPPORT_BOTH:
            Eigen::Matrix4f rightPose = rightFoot->getGlobalPose();
            Eigen::Matrix4f leftPose  = leftFoot->getGlobalPose();
            refToWorld.block(0, 3, 3, 1) = (rightPose.block(0, 3, 3, 1) + leftPose.block(0, 3, 3, 1)) / 2.0;
            Eigen::Vector3f zAxis(0, 0, 1);
            Eigen::Vector3f xAxis = (rightPose.block(0, 0, 3, 1) + leftPose.block(0, 0, 3, 1)) / 2.0;
            xAxis /= xAxis.norm();
            Eigen::Vector3f yAxis = zAxis.cross(xAxis);
            yAxis /= yAxis.norm();
            refToWorld.block(0, 0, 3, 1) = xAxis;
            refToWorld.block(0, 1, 3, 1) = yAxis;
            refToWorld.block(0, 2, 3, 1) = zAxis;
            break;
    }

    return refToWorld;
}

/*
 * Transforms trajectory to ground frame.
 *
 * Both trajectory and relativeTrajectory are in m.
 */
inline void transformTrajectoryToGroundFrame(VirtualRobot::RobotPtr robot,
                                      const Eigen::Matrix3Xf& leftFootTrajectory,
                                      VirtualRobot::RobotNodePtr leftFoot,
                                      VirtualRobot::RobotNodePtr rightFoot,
                                      VirtualRobot::RobotNodeSetPtr bodyJoints,
                                      const Eigen::MatrixXf& bodyTrajectory,
                                      const Eigen::Matrix3Xf& trajectory,
                                      const std::vector<SupportPhase>& phase,
                                      Eigen::Matrix3Xf& relativeTrajectory)
{
    Eigen::Matrix4f leftInitialPose = bodyJoints->getKinematicRoot()->getGlobalPose();
    int N = trajectory.cols();
    relativeTrajectory.resize(3, N);
    for(int i = 0; i < N; i++)
    {
        // Move basis along with the left foot
        Eigen::Matrix4f leftFootPose = leftInitialPose;
        leftFootPose.block(0,3,3,1) = 1000 * leftFootTrajectory.col(i);
        robot->setGlobalPose(leftFootPose);
        bodyJoints->setJointValues(bodyTrajectory.col(i));
        Eigen::Matrix4f worldToRef = computeGroundFrame(leftFoot, rightFoot, phase[i]);
        Eigen::Vector4f homVec;
        homVec(3, 0) = 1;
        homVec.block(0, 0, 3, 1) = trajectory.col(i) * 1000;
        relativeTrajectory.col(i) = worldToRef.colPivHouseholderQr().solve(homVec).block(0, 0, 3, 1) / 1000.0;
    }
}

/**
 * Compute IK to reach desired CoM and right foot pose.
 */
inline void computeStepConfiguration(VirtualRobot::RobotNodeSetPtr nodeSetJoints,
                              VirtualRobot::RobotNodeSetPtr nodeSetBodies,
                              VirtualRobot::RobotNodePtr waist,
                              const Eigen::Vector3f &targetCoM,
                              const Eigen::Vector3f &targetRightFoot,
                              const Eigen::Matrix4f &initialRightFootPose,
                              Eigen::VectorXf &result)
{
    const float ikPrec = 0.1;

    std::vector<VirtualRobot::HierarchicalIK::JacobiDefinition> jacobiDefinitions;

    // TCP constraint for right foot pose
    VirtualRobot::DifferentialIKPtr diffIK;
    diffIK.reset(new VirtualRobot::DifferentialIK(nodeSetJoints));
    Eigen::Matrix4f footGoal = initialRightFootPose;
    footGoal.block(0,3,3,1) = targetRightFoot;
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
    for(int i = 0; i < 30; i++)
    {
        float e = 0;
        for(int j = 0; j < jacobiDefinitions.size(); j++)
        {
            e += jacobiDefinitions[j].jacProvider->getError().norm();
        }

        if(e > 2*lastErrorLength)
        {
            break;
        }
        else if(e < ikPrec)
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
    for(int i = 0; i < jacobiDefinitions.size(); i++)
    {
        float e = jacobiDefinitions[i].jacProvider->getError().norm();

        std::cout << "e_" << i << "=" << e << "   ";

        if(e > ikPrec)
        {
            ok = false;
        }
    }
    std::cout << ((e1 <= ikPrec && e2 <= ikPrec)? "IK Ok" : "IK Failed") << std::endl;

    nodeSetJoints->getJointValues(result);
}

/*
 * Set correct initial robot position *before* calling this function.
 * For example, extend the arms so they won't collide with the body.
 *
 * Expects the Left foot as root of the kinematic chain.
 * Right foot should be the TCP.
 * trajectory is the computed walking trajectory.
 */
inline void computeWalkingTrajectory(VirtualRobot::RobotPtr robot,
                              const Eigen::Matrix3Xf& comTrajectory,
                              const Eigen::Matrix3Xf& rightFootTrajectory,
                              const Eigen::Matrix3Xf& leftFootTrajectory,
                              Eigen::MatrixXf& trajectory)
{
    std::string nodeSetName = "Left2RightLeg";
    std::string colModelName = "ColModelAll";
	std::string waistNodeName = "Waist";

    VirtualRobot::RobotNodeSetPtr nodeSet = robot->getRobotNodeSet(nodeSetName);
    VirtualRobot::RobotNodeSetPtr colModelNodeSet = robot->getRobotNodeSet(colModelName);
	VirtualRobot::RobotNodePtr waist = robot->getRobotNode(waistNodeName);

    int rows = nodeSet->getSize();

    trajectory.resize(rows, rightFootTrajectory.cols());

    Eigen::Matrix4f rightInitialPose = nodeSet->getTCP()->getGlobalPose();
    Eigen::Matrix4f leftInitialPose = nodeSet->getKinematicRoot()->getGlobalPose();

    Eigen::Vector3f com = colModelNodeSet->getCoM();

    Eigen::VectorXf configuration;
    int N = trajectory.cols();
    for(int i = 0; i < N; i++)
    {
        // Move basis along with the left foot
        Eigen::Matrix4f leftFootPose = leftInitialPose;
        leftFootPose.block(0,3,3,1) = 1000 * leftFootTrajectory.col(i);
        robot->setGlobalPose(leftFootPose);

        std::cout << "Frame #" << i << ", ";
        computeStepConfiguration(nodeSet, colModelNodeSet, waist, 1000 * comTrajectory.col(i), 1000 * rightFootTrajectory.col(i), rightInitialPose, configuration);

        //Eigen::Vector3f delta = Eigen::Vector3f(0, 0, 100 * sin(2 * 3.141 * i / (float)N));
        //computeStepConfiguration(nodeSetName, colModelName, com + delta, 1000 * rightTrajectory.col(0), rightInitialPose, configuration);

        trajectory.col(i) = configuration;
    }
}

}

#endif
