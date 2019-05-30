//
// Created by androst on 04.10.18.
//

#ifndef ECHOBOT_ROBOTINTERFACE_H
#define ECHOBOT_ROBOTINTERFACE_H

#include "SensorInterface.h"
#include <corah/Robot.h>

namespace echobot
{

class RobotInterface : public SensorInterface
{
    ECHOBOT_OBJECT(RobotInterface)

    public:
        RobotInterface();
        ~RobotInterface(){};

        SharedPointer<corah::Robot> robot;
};

}

#endif //ECHOBOT_ROBOTINTERFACE_H
