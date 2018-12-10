project_add_sources(
        main.cpp
        ApplicationGUI.cpp
        ApplicationGUI.hpp
        CameraInterface.cpp
        CameraInterface.hpp
        RobotInterface.cpp
        RobotInterface.h
        UltrasoundInterface.cpp
        UltrasoundInterface.hpp
)

project_add_subdirectories(widgets)
project_add_subdirectories(visualization)