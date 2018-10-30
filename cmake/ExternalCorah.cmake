# Download and set up Eigen

include(cmake/Externals.cmake)

ExternalProject_Add(corah
        PREFIX ${FASTROMO_EXTERNAL_BUILD_DIR}/corah
        BINARY_DIR ${FASTROMO_EXTERNAL_BUILD_DIR}/corah
        GIT_REPOSITORY "ssh://git@github.com/androst/corah.git"
        GIT_TAG "master"
        INSTALL_DIR ${FASTROMO_EXTERNAL_INSTALL_DIR}
        CMAKE_CACHE_ARGS
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DCMAKE_VERBOSE_MAKEFILE:BOOL=OFF
            -DCMAKE_INSTALL_MESSAGE:BOOL=LAZY
            -DCMAKE_INSTALL_PREFIX:PATH=${FASTROMO_EXTERNAL_INSTALL_DIR}
            -DBUILD_TESTING:BOOL=OFF
            -DDOWNLOAD_AND_BUILD_QT5:BOOL=ON
        )

if(WIN32)
    set(CORAH_LIBRARY corah.lib)
else()
    set(CORAH_LIBRARY ${CMAKE_SHARED_LIBRARY_PREFIX}corah${CMAKE_SHARED_LIBRARY_SUFFIX})
endif()

list(APPEND LIBRARIES ${CORAH_LIBRARY})
list(APPEND FASTROMO_EXTERNAL_DEPENDENCIES corah)