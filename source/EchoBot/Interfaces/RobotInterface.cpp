//
// Created by androst on 04.10.18.
//

#include "RobotInterface.h"

namespace echobot
{

RobotInterface::RobotInterface() {
    robot = std::shared_ptr<corah::Robot>(new corah::Robot);
}

RobotInterface::~RobotInterface() {

}

}