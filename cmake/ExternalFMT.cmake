include(cmake/Externals.cmake)

ExternalProject_Add(fmt
        PREFIX ${ECHOBOT_EXTERNAL_BUILD_DIR}/fmt
        BINARY_DIR ${ECHOBOT_EXTERNAL_BUILD_DIR}/fmt
        GIT_REPOSITORY "https://github.com/fmtlib/fmt.git"
        GIT_TAG "6.1.2"
        CMAKE_CACHE_ARGS
            -DBUILD_SHARED_LIBS:BOOL=TRUE
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DCMAKE_VERBOSE_MAKEFILE:BOOL=OFF
            -DCMAKE_INSTALL_MESSAGE:BOOL=LAZY
            -DCMAKE_INSTALL_PREFIX:PATH=${ECHOBOT_EXTERNAL_INSTALL_DIR}
        )

if(WIN32)
    set(FMT_LIBRARY fmt.lib)
else()
    set(FMT_LIBRARY ${CMAKE_SHARED_LIBRARY_PREFIX}fmt${CMAKE_SHARED_LIBRARY_SUFFIX})
    set(FMT_INCLUDE_DIRS ${ECHOBOT_EXTERNAL_BUILD_DIR}/fmt/include ${ECHOBOT_EXTERNAL_BUILD_DIR}/fmt ${ECHOBOT_EXTERNAL_BUILD_DIR}/fmt/src/fmt/source)
    set(FMT_LIBRARY_DIR ${ECHOBOT_EXTERNAL_BUILD_DIR}/fmt/lib)
    link_directories(${FMT_LIBRARY_DIR})
endif()

list(APPEND ECHOBOT_INCLUDE_DIRS ${FMT_INCLUDE_DIRS})
list(APPEND LIBRARIES ${FMT_LIBRARY})
list(APPEND ECHOBOT_EXTERNAL_DEPENDENCIES fmt)