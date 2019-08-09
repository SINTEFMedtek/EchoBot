#include "ApplicationGUI.hpp"

using namespace echobot;

int main() {
    ApplicationGUI::pointer window = ApplicationGUI::New();
    window->start();
}