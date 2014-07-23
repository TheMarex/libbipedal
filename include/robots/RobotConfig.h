#ifndef __ROBOT_CONFIG_H__
#define __ROBOT_CONFIG_H__

namespace Bipedal
{
struct RobotConfig
{
	const char* ROBOT_NAME;
	const char* WAIST_NODE_NAME;
	const char* COL_NODESET_NAME;
	const char* ROBOT_NODESET_NAME;
	const char* LEGS_NODESET_NAME;
	const char* CHEST_NODE_NAME;
	const char* LEGS_IK_CHAIN;
	const char* LEFT_LEG_TCP_NODE_NAME;
	const char* RIGHT_LEG_TCP_NODE_NAME;
	const char* LEFT_ANKLE_NODE_NAME;
	const char* RIGHT_ANKLE_NODE_NAME;
	const char* LEFT_ANKLE_SENSOR_NAME;
	const char* RIGHT_ANKLE_SENSOR_NAME;
};
}

#endif

