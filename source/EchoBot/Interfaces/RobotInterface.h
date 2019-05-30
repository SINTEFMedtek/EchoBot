//
// Created by androst on 04.10.18.
//

#ifndef ECHOBOT_ROBOTINTERFACE_H
#define ECHOBOT_ROBOTINTERFACE_H

#include <iostream>
#include <memory>
#include <QObject>

#include <corah/Robot.h>

namespace echobot
{
    typedef std::shared_ptr<class RobotInterface> RobotInterfacePtr;

    class RobotInterface
    {
        public:
            RobotInterface();
            ~RobotInterface();

            std::shared_ptr<corah::Robot> robot;
    };

}

#endif //ECHOBOT_ROBOTINTERFACE_H
