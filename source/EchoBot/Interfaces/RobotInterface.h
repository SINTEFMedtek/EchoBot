//
// Created by androst on 04.10.18.
//

#ifndef FASTROMO_ROBOTINTERFACE_H
#define FASTROMO_ROBOTINTERFACE_H

#include <iostream>
#include <memory>
#include <QObject>

#include <corah/Robot.h>

typedef std::shared_ptr<class RobotInterface> RobotInterfacePtr;

class RobotInterface
{
    public:
        RobotInterface();
        ~RobotInterface();

        std::shared_ptr<corah::Robot> robot;
};


#endif //FASTROMO_ROBOTINTERFACE_H
