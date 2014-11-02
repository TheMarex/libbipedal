/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __ROBOT_CONFIG_H__
#define __ROBOT_CONFIG_H__

namespace Bipedal
{
struct RobotConfig
{
	const char* ROBOT_NAME;
    // All bodies with weight
	const char* COL_NODESET_NAME;
    // All nodes that should be considered for an IK
	const char* ROBOT_NODESET_NAME;
    // All nodes that are needed for walking (e.g. arms/head not included)
	const char* WALKING_NODESET_NAME;
    // Both legs
	const char* LEGS_NODESET_NAME;
	const char* LEFT_LEG_NODESET_NAME;
	const char* RIGHT_LEG_NODESET_NAME;
    // Node should come before torso joints
	const char* WAIST_NODE_NAME;
    // Should come after torso joints
	const char* CHEST_NODE_NAME;
	const char* LEFT_LEG_TCP_NODE_NAME;
	const char* RIGHT_LEG_TCP_NODE_NAME;
	const char* LEFT_ANKLE_NODE_NAME;
	const char* RIGHT_ANKLE_NODE_NAME;
    // Name of ankle torque sensors
	const char* LEFT_ANKLE_X_SENSOR_NAME;
	const char* RIGHT_ANKLE_X_SENSOR_NAME;
	const char* LEFT_ANKLE_Y_SENSOR_NAME;
	const char* RIGHT_ANKLE_Y_SENSOR_NAME;
    // Name of foot contact sensors
	const char* LEFT_FOOT_CONTACT_SENSOR_NAME;
	const char* RIGHT_FOOT_CONTACT_SENSOR_NAME;
    // Body that is in contact with floor for computing the support polygone
	const char* LEFT_FOOT_BODY_NAME;
	const char* RIGHT_FOOT_BODY_NAME;
    // Body after the food model
	const char* LEFT_ANKLE_BODY_NAME;
	const char* RIGHT_ANKLE_BODY_NAME;
};
}

#endif

