//
// Created by androst on 04.10.18.
//

#include "RobotInterface.h"

namespace echobot
{

RobotInterface::RobotInterface() {
    mRobot = SharedPointer<romocc::Robot>(new romocc::Robot);
}

RobotInterface::~RobotInterface() {
    mRobot->disconnect();
}

void RobotInterface::setConfiguration(romocc::Manipulator manipulator, const std::string& ip_address, const int& port){
    mManipulator = manipulator;
    mHost = ip_address;
    mPort = port;
}

void RobotInterface::connect(){
    mRobot->addUpdateSubscription(std::bind(&RobotInterface::stateUpdated, this));
    mRobot->configure(mManipulator, mHost, mPort);
    mRobot->connect();
}

void RobotInterface::disconnect(){
    mRobot->disconnect();
}

bool RobotInterface::isConnected()
{
    return mRobot->isConnected();
}

RobotState::pointer RobotInterface::getCurrentState()
{
    return mRobot->getCurrentState();
}

}