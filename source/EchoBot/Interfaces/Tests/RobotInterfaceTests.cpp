//
// Created by androst on 06.08.19.
//

#include "EchoBot/Tests/catch.hpp"
#include "EchoBot/Interfaces/RobotInterface.h"

namespace echobot
{

TEST_CASE("Create RobotInterface object", "[EchoBot][Interfaces]") {
    RobotInterface::pointer rInterface = echobot::RobotInterface::New();
}

}