#include "ApplicationGUI.hpp"
#include "RobotInterface.h"

using namespace fast;

int main() {
    ApplicationGUI::pointer window = ApplicationGUI::New();
    window->start(STREAMING_MODE_NEWEST_FRAME_ONLY);
}