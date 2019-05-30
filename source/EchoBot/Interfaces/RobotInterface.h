//
// Created by androst on 04.10.18.
//

#ifndef ECHOBOT_ROBOTINTERFACE_H
#define ECHOBOT_ROBOTINTERFACE_H

#include <iostream>
#include <memory>
#include <QObject>

#include <EchoBot/Core/SmartPointers.h>
#include <corah/Robot.h>
#include "FAST/ProcessObject.hpp"

namespace echobot
{
using namespace fast;

//typedef std::shared_ptr<class RobotInterface> RobotInterfacePtr;

class RobotInterface
{
    ECHOBOT_OBJECT(RobotInterface)

    public:
        RobotInterface();
        ~RobotInterface();
        std::shared_ptr<corah::Robot> robot;

    private:
        std::weak_ptr<RobotInterface> mPtr;
};

}

#endif //ECHOBOT_ROBOTINTERFACE_H
