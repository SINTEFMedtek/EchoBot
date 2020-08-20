#include "EchoBot/Tests/catch.hpp"
#include "EchoBot/Utilities/PointCloudUtilities.h"
#include "EchoBot/Exporters/PointCloudExporter.h"
#include "EchoBot/Core/Config.h"

#include "EchoBot/Interfaces/Robot/RobotInterface.h"
#include "RobotVisualization.h"
#include "VisualizationHelper.h"

#include "FAST/Visualization/SimpleWindow.hpp"



namespace echobot {

TEST_CASE("Visualize robot joints", "[EchoBot][Visualization]") {
    auto robotInterface = RobotInterface::New();
    robotInterface->setConfiguration(ManipulatorType::UR10, "192.168.153.131", 30003);
    robotInterface->connect();

    auto robotVisualizator = RobotVisualizator::New();
    robotVisualizator->setInterface(robotInterface);

    auto refMbase = Eigen::Affine3f::Identity();
    auto bMee = robotInterface->getCurrentState()->get_bMee().cast<float>();

    auto window = fast::SimpleWindow::New();
    window->set3DMode();
    window->addRenderer(robotVisualizator->getRenderer());
    window->addRenderer(VisualizationHelper::createCoordinateFrameRenderer(refMbase, 300));
    window->addRenderer(VisualizationHelper::createCoordinateFrameRenderer(bMee, 300));
    window->start();
}

}