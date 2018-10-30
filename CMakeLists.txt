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

target_include_directories(fastromo
        PUBLIC
            ${FASTROMO_INCLUDE_DIRS}
            ${FASTROMO_EXTERNAL_INSTALL_DIR}/include
            corah
            /home/androst/dev/ROMO/FAST-ROMO/build/external/corah/include/kdl
            /home/androst/dev/ROMO/FAST-ROMO/build/external/corah
            /home/androst/dev/ROMO/FAST-ROMO/build/external/corah/src/corah/corah
            /home/androst/dev/ROMO/FAST-ROMO/build/external/corah/include/QtNetwork
            /home/androst/dev/ROMO/FAST-ROMO/build/external/corah/include/QtWidgets
        )

add_executable(aorticAneurysmExam AorticAneurysmExam/main.cpp)
target_link_libraries(aorticAneurysmExam fastromo)