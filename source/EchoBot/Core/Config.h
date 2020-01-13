//
// Created by androst on 12.01.20.
//

#ifndef ECHOBOT_CONFIG_H
#define ECHOBOT_CONFIG_H

#include <string>
#include <EchoBotExport.hpp>

namespace echobot{

namespace Config{
    ECHOBOT_EXPORT std::string getRegistrationDataPath();
    ECHOBOT_EXPORT std::string getTestDataPath();
}


}

#endif //ECHOBOT_CONFIG_H
