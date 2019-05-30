//
// Created by androst on 30.05.19.
//

#ifndef ECHOBOT_SENSORINTERFACE_H
#define ECHOBOT_SENSORINTERFACE_H

#include <EchoBot/Core/SmartPointers.h>

namespace echobot
{

class SensorInterface {
    ECHOBOT_OBJECT(SensorInterface)

    protected:
        std::weak_ptr<SensorInterface> mPtr;
};

}


#endif //ECHOBOT_SENSORINTERFACE_H
