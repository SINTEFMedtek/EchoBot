//
// Created by androst on 12.01.20.
//

#include "Utilities.h"

namespace echobot{

std::vector<std::string> split(const std::string input, const std::string &delimiter) {
    std::vector<std::string> parts;
    int startPos = 0;
    while(true) {
        int pos = input.find(delimiter, startPos);
        if(pos == std::string::npos) {
            parts.push_back(input.substr(startPos));
            break;
        }

        if(pos - startPos > 0)
            parts.push_back(input.substr(startPos, pos - startPos));
        startPos = pos + delimiter.length();
    }

    return parts;
}

std::string replace(std::string str, std::string find, std::string replacement) {
    while(true) {
        int pos = str.find(find);
        if(pos == std::string::npos)
            break;
        str.replace(pos, find.size(), replacement);
    }

    return str;
}

}