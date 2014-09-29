#ifndef __STABILIZER_FACTORY_H__
#define __STABILIZER_FACTORY_H__

#include <VirtualRobot/VirtualRobot.h>

#include "../bipedal.h"

namespace Bipedal
{

class StabilizerFactory
{
public:
    StabilizerFactory(const Bipedal::RobotConfig& config, const VirtualRobot::RobotPtr& robot);

    KajitaStabilizerPtr kajitaStab;
    CartesianStabilizerPtr cartesianStab;
};

}

#endif
