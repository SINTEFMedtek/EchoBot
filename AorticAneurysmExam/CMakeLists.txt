project_add_sources(
        main.cpp
        ApplicationGUI.cpp
        ApplicationGUI.hpp
        KinectTracking.cpp
        KinectTracking.hpp
        RobotInterface.cpp
        RobotInterface.h
)

project_add_subdirectories(widgets)
project_add_subdirectories(visualization)