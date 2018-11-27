cmake_minimum_required(VERSION 2.8.10)
project(fastromo)

include(cmake/Macros.cmake)

# Enable C++ 11
set(CMAKE_CXX_STANDARD 11)

# Get rid of Qt error with position independent code
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    add_definitions("-fPIC")
endif()

#### Source and includes
set(FASTROMO_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/fastromo ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR})

#### Setup all external depedencies
find_package(orocos_kdl)
include(cmake/Requirements.cmake)

#### Set include dirs
message("-- INCLUDES: ${FASTROMO_INCLUDE_DIRS}")
include_directories(${FASTROMO_INCLUDE_DIRS})

get_directory_property(hasParent PARENT_DIRECTORY)
if(hasParent)
    set(FASTROMO_INCLUDE_DIRS ${FASTROMO_INCLUDE_DIRS} PARENT_SCOPE)
endif()

#### Add all subdirs
project_add_subdirectories(AorticAneurysmExam)

message("-- Dependencies: ${FASTROMO_EXTERNAL_DEPENDENCIES}")

#### Create library and executables
add_library(fastromo SHARED ${FASTROMO_SOURCE_FILES} ${HEADERS_MOC})
add_dependencies(fastromo ${FASTROMO_EXTERNAL_DEPENDENCIES})
include(GenerateExportHeader)
generate_export_header(fastromo EXPORT_FILE_NAME fastromoExport.hpp)

## Link everything
message("-- LIBS: ${LIBRARIES} ${FASTROMO_SYSTEM_LIBRARIES}")

target_link_libraries(fastromo PUBLIC ${LIBRARIES} ${FAST_LIBRARIES} orocos-kdl)

message("-- INCLUDE directories: ${FASTROMO_EXTERNAL_INSTALL_DIR}")

target_include_directories(fastromo
        PUBLIC
            ${FASTROMO_INCLUDE_DIRS}
            ${FASTROMO_EXTERNAL_INSTALL_DIR}/external/corah
            ${FASTROMO_EXTERNAL_INSTALL_DIR}/external/corah/src/corah/corah
            ${FASTROMO_EXTERNAL_INSTALL_DIR}/external/corah/include/kdl
            ${FASTROMO_EXTERNAL_INSTALL_DIR}/external/corah/include/QtNetwork
        )

add_custom_command(TARGET fastromo PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/AorticAneurysmExam/visualization/CADModels
        $<TARGET_FILE_DIR:fastromo>/AorticAneurysmExam/visualization/CADModels)

add_custom_command(TARGET fastromo PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/AorticAneurysmExam/widgets/icons
        $<TARGET_FILE_DIR:fastromo>/AorticAneurysmExam/widgets/icons)

add_executable(aorticAneurysmExam AorticAneurysmExam/main.cpp)
target_link_libraries(aorticAneurysmExam fastromo)