//
// Created by androst on 04.10.18.
//

#ifndef ECHOBOT_ROBOTINTERFACE_H
#define ECHOBOT_ROBOTINTERFACE_H

#include <QObject>

#include "EchoBot/Interfaces/SensorInterface.h"
#include <romocc/Robot.h>

namespace echobot
{

class RobotInterface : public QObject, public SensorInterface
{
    Q_OBJECT
    ECHOBOT_OBJECT(RobotInterface)

    public:
        RobotInterface();
        SharedPointer<romocc::Robot> robot;

    signals:
        void stateUpdated();
};

}

#endif //ECHOBOT_ROBOTINTERFACE_H
