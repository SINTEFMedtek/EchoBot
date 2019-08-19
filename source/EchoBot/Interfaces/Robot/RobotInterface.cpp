//
// Created by androst on 04.10.18.
//

#include "RobotInterface.h"

namespace echobot
{

RobotInterface::RobotInterface() {
    robot = SharedPointer<romocc::Robot>(new romocc::Robot);
    robot->addUpdateSubscription(std::bind(&RobotInterface::stateUpdated, this));
}

}