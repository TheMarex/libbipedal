#ifndef __ZMP_FALL_DETECTOR_H__
#define __ZMP_FALL_DETECTOR_H__

#include "../bipedal.h"
#include "../utils/ZMP.h"
#include "FallDetector.h"

#include <Eigen/Dense>

#include <VirtualRobot/VirtualRobot.h>

namespace Bipedal
{

/**
 * This class is an interface to detect instability for a given trajectory
 * based on the ZMP being contained in the support polygone.
 */
class ZMPFallDetector : public FallDetector
{
public:
    ZMPFallDetector(const VirtualRobot::RobotNodeSetPtr& colModelNodes,
                    const VirtualRobot::RobotNodePtr& leftFoot,
                    const VirtualRobot::RobotNodePtr& rightFoot,
                    const VirtualRobot::ContactSensorPtr& leftFootContactSensor,
                    const VirtualRobot::ContactSensorPtr& rightFootContactSensor,
                    const VirtualRobot::MathTools::ConvexHull2DPtr& leftSupportHull,
                    const VirtualRobot::MathTools::ConvexHull2DPtr& rightSupportHull);
    /**
     * dt is the time in seconds since the last call to this function
     * Needs to be called in each iteration of the control loop before
     * the call to isFalling.
     */
    virtual void update(double dt) override;

    /**
     * Returns true if it was detected that we are falling.
     */
    virtual bool isFalling() const override;

    const Eigen::Vector3f& getContactPoint() const;

private:
    bool getStabilityInidcator(SupportPhase phase);

    bool falling;
    double maxHullDist;
    unsigned minFallingFrames;
    unsigned fallingFrameCounter;
    Bipedal::SupportPhase lastSupportPhase;
    Bipedal::SupportPhaseSensorPtr supportPhaseSensor;

    VirtualRobot::RobotNodePtr leftFoot;
    VirtualRobot::RobotNodePtr rightFoot;
    VirtualRobot::MathTools::ConvexHull2DPtr leftSupportHull;
    VirtualRobot::MathTools::ConvexHull2DPtr rightSupportHull;
    VirtualRobot::MathTools::ConvexHull2DPtr dualSupportHull;

    VirtualRobot::RobotNodeSetPtr colModelNodes;
    Bipedal::CartTableZMPEstimator zmpEstimator;
    Eigen::Vector3f contactPoint;
};

}

#endif
