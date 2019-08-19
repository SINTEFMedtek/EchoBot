project_add_subdirectories(Camera)
project_add_subdirectories(Robot)
project_add_subdirectories(Ultrasound)

project_add_sources(
        SensorInterface.h
)

project_add_test_sources(
        Tests/RobotInterfaceTests.cpp
)
