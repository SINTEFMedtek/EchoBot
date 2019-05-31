cmake_minimum_required(VERSION 3.1)
project(EchoBot)

include(cmake/Macros.cmake)

#### Options
option(ECHOBOT_BUILD_QT5 "Download and build Qt. Turn OFF if you want to use pre-built binaries existing on your machine." ON)
option(ECHOBOT_BUILD_FAST "Download and build FAST." OFF)
option(ECHOBOT_BUILD_CORAH "Download and build Corah." ON)

# Enable C++ 14
set(CMAKE_CXX_STANDARD 14)

# Get rid of Qt error with position independent code
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    add_definitions("-fPIC")
endif()

#### Paths
set(ECHOBOT_TEST_DATA_DIR "" CACHE PATH "Directory of test data. Default is ROOT/data/.")
set(ECHOBOT_SOURCE_DIR "${PROJECT_SOURCE_DIR}/source/EchoBot")
set(ECHOBOT_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/source/ ${CMAKE_CURRENT_BINARY_DIR})

#### Setup all external depedencies
find_package(orocos_kdl)
include(cmake/Requirements.cmake)

#### Set include dirs
message("-- INCLUDES: ${ECHOBOT_INCLUDE_DIRS}")
include_directories(${ECHOBOT_INCLUDE_DIRS})

get_directory_property(hasParent PARENT_DIRECTORY)
if(hasParent)
    set(ECHOBOT_INCLUDE_DIRS ${ECHOBOT_INCLUDE_DIRS} PARENT_SCOPE)
endif()

#### Add all subdirs
project_add_subdirectories(source/EchoBot)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_AUTOMOC ON)

message("-- Dependencies: ${ECHOBOT_EXTERNAL_DEPENDENCIES}")

#### Create library and executables
add_library(EchoBot SHARED ${ECHOBOT_SOURCE_FILES} ${HEADERS_MOC})
add_dependencies(EchoBot ${ECHOBOT_EXTERNAL_DEPENDENCIES})

include(GenerateExportHeader)
generate_export_header(EchoBot EXPORT_FILE_NAME EchoBotExport.hpp)

## Link everything
message("-- LIBS: ${LIBRARIES}")
target_link_libraries(EchoBot PUBLIC ${LIBRARIES})

target_include_directories(EchoBot PUBLIC ${ECHOBOT_INCLUDE_DIRS})

add_custom_command(TARGET EchoBot PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/source/EchoBot/Visualization/CADModels
        $<TARGET_FILE_DIR:EchoBot>/source/EchoBot/Visualization/CADModels)

add_custom_command(TARGET EchoBot PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/source/EchoBot/GUI/Widgets/Icons
        $<TARGET_FILE_DIR:EchoBot>/source/EchoBot/GUI/Widgets/Icons)

project_add_subdirectories(applications)