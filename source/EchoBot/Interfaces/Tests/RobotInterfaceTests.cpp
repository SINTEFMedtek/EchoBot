//
// Created by androst on 06.08.19.
//

#include "EchoBot/Tests/catch.hpp"
#include "source/EchoBot/Interfaces/Robot/RobotInterface.h"

namespace echobot
{

TEST_CASE("Create RobotInterface object", "[EchoBot][Interfaces]") {
    auto robotInterface = echobot::RobotInterface::New();
}

TEST_CASE("Connect and listen to new states", "[EchoBot][Interfaces]"){
    auto robotInterface = echobot::RobotInterface::New();
    robotInterface->setConfiguration(ManipulatorType::UR10, "10.218.180.32", 30003);
    robotInterface->connect();

    for(int i = 0; i<50; i++)
    {
        std::cout << robotInterface->getCurrentState()->getJointConfig().transpose() << std::endl;
    }
}

}