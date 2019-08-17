#include "RecordApplicationGUI.hpp"

using namespace echobot;

int main() {
    RecordApplicationGUI::pointer window = RecordApplicationGUI::New();
    window->start();
}