#include <iostream>
#include <romocc/utilities/MathUtils.h>

#include "EchoBot/Tests/catch.hpp"
#include "EchoBot/Core/DataTypes.h"
#include "EchoBot/Utilities/CalibrationTool.h"

namespace echobot
{

TEST_CASE("Initialize tool and calibrate", "[EchoBot][Utilities]"){
    auto calibrationTool = CalibrationTool::New();
    calibrationTool->calibrate();

    std::cout << calibrationTool->get_rMb().matrix() << std::endl;
    std::cout << calibrationTool->get_eeMt().matrix() << std::endl;
    std::cout << calibrationTool->get_tMus().matrix() << std::endl;
}

TEST_CASE("Load calibration files.", "[EchoBot][Utilities]") {
    auto calibrationTool = CalibrationTool::New();
    auto path = Config::getConfigPath() + "calibration/camMbase.cal";

    auto calMat = CalibrationTool::loadCalFile(path);
    std::cout << path << std::endl;
    std::cout << calMat.matrix() << std::endl;

    auto vec = toVector6D(calMat);
    std::cout << vec << std::endl;
}

TEST_CASE("Save calibration files.", "[EchoBot][Utilities]") {
    auto calibrationTool = CalibrationTool::New();
    auto path = Config::getConfigPath() + "calibration/test.cal";

    auto calMat = Eigen::Affine3d::Identity();
    calMat(0,3) = 30.3;

    CalibrationTool::saveCalFile(path, calMat);
}

TEST_CASE("Load ", "[EchoBot][Utilities]") {

}

}