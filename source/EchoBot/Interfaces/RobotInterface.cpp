//
// Created by androst on 04.10.18.
//

#include "RobotInterface.h"

namespace echobot
{

RobotInterface::RobotInterface() {
    robot = SharedPointer<corah::Robot>(new corah::Robot);
}

}