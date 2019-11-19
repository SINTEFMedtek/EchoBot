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

using namespace romocc;

class RobotInterface : public QObject, public SensorInterface
{
    Q_OBJECT
    ECHOBOT_OBJECT(RobotInterface)

    public:
        RobotInterface();
        Robot::pointer getRobot(){ return mRobot;};

        void connect();
        void disconnect();
        bool isConnected();

        void setConfiguration(romocc::Manipulator manipulator, const std::string& ip_address, const int& port);

        RobotState::pointer getCurrentState();

    private:
        SharedPointer<romocc::Robot> mRobot;
        Manipulator mManipulatorType;
        std::string mHost;
        int mPort;


    signals:
        void stateUpdated();
};

}

#endif //ECHOBOT_ROBOTINTERFACE_H
