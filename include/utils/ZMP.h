#ifndef __UTILS_ZMP_H__
#define __UTILS_ZMP_H__

namespace Bipedal
{
    /*
     * Computes ZMP according to the table cart model.
     */
    Eigen::Vector2f computeModelZMP(Eigen::Vector3f com, Eigen::Vector3f comAcc, Eigen::Vector3f gravity)
    {
        Eigen::Vector2f zmp;
        // note that gravity.z() < 0, so we changed the sign of the term
        zmp.x() = com.x() + com.z() / gravity.z() * comAcc.x();
        zmp.y() = com.y() + com.z() / gravity.z() * comAcc.y();
        return zmp;
    }
};

#endif
