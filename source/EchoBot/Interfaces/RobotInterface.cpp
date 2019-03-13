//
// Created by androst on 04.10.18.
//

#include "RobotInterface.h"

RobotInterface::RobotInterface() {
    robot = std::shared_ptr<corah::Robot>(new corah::Robot);
}

RobotInterface::~RobotInterface() {

}