# Download and set up Eigen

include(cmake/Externals.cmake)

ExternalProject_Add(corah
        PREFIX ${ECHOBOT_EXTERNAL_BUILD_DIR}/corah
        BINARY_DIR ${ECHOBOT_EXTERNAL_BUILD_DIR}/corah
        GIT_REPOSITORY "ssh://git@github.com/androst/corah.git"
        GIT_TAG "master"
        INSTALL_DIR ${ECHOBOT_EXTERNAL_INSTALL_DIR}
        CMAKE_CACHE_ARGS
            -DCMAKE_PREFIX_PATH:PATH=${ECHOBOT_EXTERNAL_INSTALL_DIR}/lib/cmake/Qt5
            -DCMAKE_PLUGIN_DIR:PATH=${ECHOBOT_EXTERNAL_INSTALL_DIR}/plugins
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DCMAKE_VERBOSE_MAKEFILE:BOOL=OFF
            -DCMAKE_INSTALL_MESSAGE:BOOL=LAZY
            -DCMAKE_INSTALL_PREFIX:PATH=${ECHOBOT_EXTERNAL_INSTALL_DIR}
            -DBUILD_TESTING:BOOL=OFF
            -DDOWNLOAD_AND_BUILD_QT5:BOOL=ON
        )

if(WIN32)
    set(CORAH_LIBRARY corah.lib)
else()
    set(CORAH_LIBRARY ${CMAKE_SHARED_LIBRARY_PREFIX}corah${CMAKE_SHARED_LIBRARY_SUFFIX})
endif()

list(APPEND ECHOBOT_INCLUDE_DIRS ${install_dir}/fast/include)
list(APPEND LIBRARIES ${CORAH_LIBRARY})
list(APPEND ECHOBOT_EXTERNAL_DEPENDENCIES corah)