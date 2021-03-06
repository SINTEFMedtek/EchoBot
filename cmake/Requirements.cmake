# Setup all dependencies, both internal (have to be installed on the system)
# and external (downloaded and built automatically)

## Qt
if(ECHOBOT_BUILD_QT5)
    include(cmake/ExternalQt5.cmake)
    # Use Project Qt CMake files
    set(Qt5Core_DIR ${PROJECT_SOURCE_DIR}/cmake/Qt5Core)
    set(Qt5Gui_DIR ${PROJECT_SOURCE_DIR}/cmake/Qt5Gui)
    set(Qt5Widgets_DIR ${PROJECT_SOURCE_DIR}/cmake/Qt5Widgets)
    set(Qt5OpenGL_DIR ${PROJECT_SOURCE_DIR}/cmake/Qt5OpenGL)
    set(Qt5Multimedia_DIR ${PROJECT_SOURCE_DIR}/cmake/Qt5Multimedia)
    set(Qt5MultimediaWidgets_DIR ${PROJECT_SOURCE_DIR}/cmake/Qt5MultimediaWidgets)
    set(Qt5Network_DIR ${PROJECT_SOURCE_DIR}/cmake/Qt5Network)
    find_package(Qt5Widgets REQUIRED PATHS ${PROJECT_SOURCE_DIR}/cmake/)
    find_package(Qt5OpenGL REQUIRED PATHS ${PROJECT_SOURCE_DIR}/cmake/)
    find_package(Qt5Multimedia REQUIRED PATHS ${PROJECT_SOURCE_DIR}/cmake/)
    find_package(Qt5MultimediaWidgets REQUIRED PATHS ${PROJECT_SOURCE_DIR}/cmake/)
    find_package(Qt5Network REQUIRED PATHS ${PROJECT_SOURCE_DIR}/cmake/)
    list(APPEND LIBRARIES ${Qt5Core_LIBRARY})
    list(APPEND LIBRARIES ${Qt5Gui_LIBRARY})
    list(APPEND LIBRARIES ${Qt5Widgets_LIBRARY})
    list(APPEND LIBRARIES ${Qt5OpenGL_LIBRARY})
    list(APPEND LIBRARIES ${Qt5Multimedia_LIBRARY})
    list(APPEND LIBRARIES ${Qt5MultimediaWidgets_LIBRARY})
    list(APPEND LIBRARIES ${Qt5Network_LIBRARY})
else(ECHOBOT_BUILD_QT5)
    find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets OpenGL Multimedia MultimediaWidgets Network)
    list(APPEND LIBRARIES Qt5::Core)
    list(APPEND LIBRARIES Qt5::Gui)
    list(APPEND LIBRARIES Qt5::Widgets)
    list(APPEND LIBRARIES Qt5::OpenGL)
    list(APPEND LIBRARIES Qt5::Multimedia)
    list(APPEND LIBRARIES Qt5::MultimediaWidgets)
    list(APPEND LIBRARIES Qt5::Network)
endif(ECHOBOT_BUILD_QT5)

list(APPEND ECHOBOT_INCLUDE_DIRS ${Qt5Widgets_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS}
        ${Qt5OpenGL_INCLUDE_DIRS} ${Qt5Multimedia_INCLUDE_DIRS} ${Qt5MultimediaWidgets_INCLUDE_DIRS} ${Qt5Network_INCLUDE_DIRS})

if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    add_definitions("-fPIC") # Get rid of Qt error with position independent code
endif()

qt5_wrap_cpp(HEADERS_MOC ${QT_HEADERS})

## External depedencies

# FAST
if(ECHOBOT_BUILD_FAST)
    include(cmake/ExternalFAST.cmake)
else(ECHOBOT_BUILD_FAST)
    find_package(FAST REQUIRED)
    list(APPEND LIBRARIES ${FAST_LIBRARIES})
    list(APPEND ECHOBOT_INCLUDE_DIRS ${FAST_INCLUDE_DIRS})
    list(APPEND ECHOBOT_EXTERNAL_LIBRARIES ${FAST_LIBRARY_DIRS})
endif(ECHOBOT_BUILD_FAST)

# Clarius streaming
if(ECHOBOT_ENABLE_CLARIUS_STREAMING)
    # User has to supply the path to the claris sdk
    set(CLARIUS_SDK_DIR "NOT_SET" CACHE PATH "Path to the clarius listen API.")
    if(${CLARIUS_SDK_DIR} STREQUAL "NOT_SET")
        message(FATAL_ERROR "Clarius ultrasound module was enabled, but Clarius SDK dir has not been set in CMake")
    else()
        message(STATUS "Clarius ultrasound module enabled. Clarius SDK dir set to: ${CLARIUS_SDK_DIR}")
    endif()

    list(APPEND ECHOBOT_INCLUDE_DIRS ${CLARIUS_SDK_DIR}/include)
    if(WIN32)
        list(APPEND LIBRARIES ${CLARIUS_SDK_DIR}/lib/listen.lib)
    else()
        list(APPEND LIBRARIES liblisten.so)
        link_directories(${CLARIUS_SDK_DIR}/lib/)
    endif()
endif()

# Romocc
if(ECHOBOT_BUILD_ROMOCC)
    include(cmake/ExternalRomocc.cmake)
else(ECHOBOT_BUILD_ROMOCC)
    find_package(Romocc REQUIRED)
    list(APPEND LIBRARIES ${ROMOCC_LIBRARIES})
    list(APPEND ECHOBOT_INCLUDE_DIRS ${ROMOCC_INCLUDE_DIRS})
    list(APPEND ECHOBOT_EXTERNAL_LIBRARIES ${ROMOCC_LIBRARY_DIRS})
endif(ECHOBOT_BUILD_ROMOCC)

# Romocc
if(ECHOBOT_BUILD_FMT)
    include(cmake/ExternalFMT.cmake)
endif(ECHOBOT_BUILD_FMT)

# Make sure project can find external includes and libaries
link_directories(${ECHOBOT_EXTERNAL_INSTALL_DIR}/lib/ ${ECHOBOT_EXTERNAL_LIBRARIES})
list(APPEND ECHOBOT_INCLUDE_DIRS ${ECHOBOT_EXTERNAL_INSTALL_DIR}/include)