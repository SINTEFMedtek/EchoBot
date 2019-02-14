# Download and set up Eigen

include(cmake/Externals.cmake)

ExternalProject_Add(fast
        PREFIX ${ECHOBOT_EXTERNAL_BUILD_DIR}/fast
        BINARY_DIR ${ECHOBOT_EXTERNAL_BUILD_DIR}/fast/build
        GIT_REPOSITORY "https://github.com/smistad/FAST.git"
        GIT_TAG "development"
        INSTALL_DIR ${ECHOBOT_EXTERNAL_INSTALL_DIR}
        CMAKE_CACHE_ARGS
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DCMAKE_VERBOSE_MAKEFILE:BOOL=OFF
            -DCMAKE_INSTALL_MESSAGE:BOOL=LAZY
            -DCMAKE_INSTALL_PREFIX:PATH=${ECHOBOT_EXTERNAL_INSTALL_DIR}
            -DBUILD_TESTING:BOOL=OFF
        )

list(APPEND LIBRARIES ${FAST_LIBRARIES})
list(APPEND ECHOBOT_EXTERNAL_DEPENDENCIES fast)