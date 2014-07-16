#ifndef __REFERENCE_IK_H__
#define __REFERENCE_IK_H__

#include <Eigen/Dense>

namespace Kinematics {
    enum SupportPhase;
};

/* Interface to compute IK from reference frames */
class ReferenceIK
{
public:
    virtual bool computeStep(const Eigen::Matrix4f& leftFootPose,
                             const Eigen::Matrix4f& rightFootPose,
                             const Eigen::Matrix4f& chestPose,
                             const Eigen::Matrix4f& pelvisPose,
                             Kinematics::SupportPhase phase,
                             Eigen::VectorXf &result) = 0;
};

#endif

