# Download and set up Eigen

include(cmake/Externals.cmake)

ExternalProject_Add(fast
        PREFIX ${FASTROMO_EXTERNAL_BUILD_DIR}/fast
        BINARY_DIR ${FASTROMO_EXTERNAL_BUILD_DIR}/fast/build
        GIT_REPOSITORY "https://github.com/smistad/FAST.git"
        GIT_TAG "development"
        INSTALL_DIR ${FASTROMO_EXTERNAL_INSTALL_DIR}
        CMAKE_CACHE_ARGS
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DCMAKE_VERBOSE_MAKEFILE:BOOL=OFF
            -DCMAKE_INSTALL_MESSAGE:BOOL=LAZY
            -DCMAKE_INSTALL_PREFIX:PATH=${FASTROMO_EXTERNAL_INSTALL_DIR}
            -DBUILD_TESTING:BOOL=OFF
        )

list(APPEND FASTROMO_INCLUDE_DIRS ${FASTROMO_EXTERNAL_INSTALL_DIR}/include/fast/)
list(APPEND FASTROMO_EXTERNAL_DEPENDENCIES fast)