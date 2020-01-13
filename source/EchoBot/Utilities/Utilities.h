//
// Created by androst on 12.01.20.
//

#ifndef ECHOBOT_UTILITIES_H
#define ECHOBOT_UTILITIES_H

#include <EchoBotExport.hpp>
#include <string>
#include <algorithm>
#include <vector>

namespace echobot{

ECHOBOT_EXPORT std::vector<std::string> split(const std::string input, const std::string& delimiter = " ");
ECHOBOT_EXPORT std::string replace(std::string str, std::string find, std::string replacement);

// trim from start (in place)
static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
                                    [](unsigned char c) {return !std::isspace(c); }));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(),
                         [](unsigned char c) {return !std::isspace(c); }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s) {
    ltrim(s);
    rtrim(s);
}

} // end namespace echobot

#endif //ECHOBOT_UTILITIES_H
