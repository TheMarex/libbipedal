#ifndef __ZMP_DISTRIBUTOR_H__
#define __ZMP_DISTRIBUTOR_H__

#include "../../bipedal.h"

#include <VirtualRobot/MathTools.h>

namespace Bipedal
{

/**
 * Implements the force distributor proposed by Kajita in his 2010 paper.
 */

class ZMPDistributor
{
public:
    struct ForceTorque
    {
        Eigen::Vector3f leftTorque;
        Eigen::Vector3f rightTorque;
        Eigen::Vector3f leftForce;
        Eigen::Vector3f rightForce;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    ZMPDistributor(double mass, Eigen::Vector3f gravity,
                     VirtualRobot::RobotNodePtr leftFoot,
                     VirtualRobot::RobotNodePtr rightFoot,
                     VirtualRobot::RobotNodePtr leftFootTCP,
                     VirtualRobot::RobotNodePtr rightFootTCP);

    Eigen::Vector2f computeHullContact(const Eigen::Matrix4f& anklePose,
                                       const Eigen::Vector3f& refZMP,
                                       const VirtualRobot::MathTools::ConvexHull2DPtr& hull);

    double computeAlpha(const Eigen::Matrix4f& groundPoseLeft,
                        const Eigen::Matrix4f& groundPoseRight,
                        const Eigen::Vector3f& refZMP,
                        const Eigen::Vector2f& refZMPLeft,
                        const Eigen::Vector2f& refZMPRight,
                        Bipedal::SupportPhase phase);

    ForceTorque distributeZMP(const Eigen::Vector3f& leftAnklePosition,
                              const Eigen::Vector3f& rightAnklePosition,
                              const Eigen::Matrix4f& leftFootTCP,
                              const Eigen::Matrix4f& rightFootTCP,
                              const Eigen::Vector3f& refZMP,
                              Bipedal::SupportPhase phase);

private:
    double mass;
    Eigen::Vector3f gravity;
    VirtualRobot::MathTools::ConvexHull2DPtr leftConvexHull;
    VirtualRobot::MathTools::ConvexHull2DPtr rightConvexHull;
};

typedef boost::shared_ptr<ZMPDistributor> ZMPDistributorPtr;

}

#endif
