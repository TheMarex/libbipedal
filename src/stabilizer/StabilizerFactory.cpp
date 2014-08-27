#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Nodes/ForceTorqueSensor.h>
#include <VirtualRobot/Robot.h>

#include "robots/RobotConfig.h"
#include "stabilizer/StabilizerFactory.h"
#include "stabilizer/KajitaStabilizer.h"
#include "stabilizer/CartesianStabilizer.h"

#include "ik/DifferentialReferenceIK.h"

#include <boost/make_shared.hpp>

StabilizerFactory::StabilizerFactory(const Bipedal::RobotConfig& config, const VirtualRobot::RobotPtr& robot)
{
    VirtualRobot::RobotNodePtr chest          = robot->getRobotNode(config.CHEST_NODE_NAME);
    VirtualRobot::RobotNodePtr leftFoot       = robot->getRobotNode(config.LEFT_LEG_TCP_NODE_NAME);
    VirtualRobot::RobotNodePtr rightFoot      = robot->getRobotNode(config.RIGHT_LEG_TCP_NODE_NAME);
    VirtualRobot::RobotNodePtr leftFootBody   = robot->getRobotNode(config.LEFT_FOOT_BODY_NAME);
    VirtualRobot::RobotNodePtr rightFootBody  = robot->getRobotNode(config.RIGHT_FOOT_BODY_NAME);
    VirtualRobot::RobotNodePtr leftAnkleBody  = robot->getRobotNode(config.LEFT_ANKLE_BODY_NAME);
    VirtualRobot::RobotNodePtr rightAnkleBody = robot->getRobotNode(config.RIGHT_ANKLE_BODY_NAME);
    VirtualRobot::RobotNodePtr pelvis         = robot->getRobotNode(config.WAIST_NODE_NAME);
    VirtualRobot::ForceTorqueSensorPtr leftAnkleSensorX
        = boost::dynamic_pointer_cast<VirtualRobot::ForceTorqueSensor>(robot->getSensor(config.LEFT_ANKLE_X_SENSOR_NAME));
    VirtualRobot::ForceTorqueSensorPtr rightAnkleSensorX
        = boost::dynamic_pointer_cast<VirtualRobot::ForceTorqueSensor>(robot->getSensor(config.RIGHT_ANKLE_X_SENSOR_NAME));
    VirtualRobot::ForceTorqueSensorPtr leftAnkleSensorY
        = boost::dynamic_pointer_cast<VirtualRobot::ForceTorqueSensor>(robot->getSensor(config.LEFT_ANKLE_Y_SENSOR_NAME));
    VirtualRobot::ForceTorqueSensorPtr rightAnkleSensorY
        = boost::dynamic_pointer_cast<VirtualRobot::ForceTorqueSensor>(robot->getSensor(config.RIGHT_ANKLE_Y_SENSOR_NAME));
    VirtualRobot::RobotNodeSetPtr nodes = robot->getRobotNodeSet(config.ROBOT_NODESET_NAME);

    auto colModel = robot->getRobotNodeSet(config.COL_NODESET_NAME);

    auto referenceIK = boost::dynamic_pointer_cast<ReferenceIK>(
                boost::make_shared<DifferentialReferenceIK>(nodes,
                                                            robot,
                                                            colModel,
                                                            leftFoot,
                                                            rightFoot,
                                                            chest,
                                                            pelvis)
              );

    kajitaStab = boost::make_shared<KajitaStabilizer>(robot,
                                                      nodes,
                                                      chest,
                                                      leftFoot, rightFoot,
                                                      leftFootBody, rightFootBody,
                                                      leftAnkleBody, rightAnkleBody,
                                                      pelvis,
                                                      leftAnkleSensorX, rightAnkleSensorX,
                                                      leftAnkleSensorY, rightAnkleSensorY,
                                                      referenceIK);

    cartesianStab = boost::make_shared<CartesianStabilizer>(robot,
                                                            nodes,
                                                            chest,
                                                            leftFoot, rightFoot,
                                                            pelvis,
                                                            referenceIK);
}
