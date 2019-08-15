# Download and set up Eigen

include(cmake/Externals.cmake)

ExternalProject_Add(romocc
        PREFIX ${ECHOBOT_EXTERNAL_BUILD_DIR}/romocc
        BINARY_DIR ${ECHOBOT_EXTERNAL_BUILD_DIR}/romocc
        GIT_REPOSITORY "https://github.com/SINTEFMedtek/libromocc.git"
        GIT_TAG "cdb991342db2794a7db85d0c2dbd5366bbfdc430"
        CMAKE_ARGS
            -DROMOCC_BUILD_TESTS:BOOL=OFF
            -DROMOCC_BUILD_EXAMPLES:BOOL=OFF
            -DROMOCC_BUILD_URSIMULATOR:BOOL=OFF
        CMAKE_CACHE_ARGS
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DCMAKE_VERBOSE_MAKEFILE:BOOL=OFF
            -DCMAKE_INSTALL_MESSAGE:BOOL=LAZY
            -DCMAKE_INSTALL_PREFIX:PATH=${ECHOBOT_EXTERNAL_INSTALL_DIR}
        )

if(WIN32)
    set(ROMOCC_LIBRARY romocc.lib)
else()
    set(ROMOCC_LIBRARY ${CMAKE_SHARED_LIBRARY_PREFIX}romocc${CMAKE_SHARED_LIBRARY_SUFFIX})
    set(ROMOCC_INCLUDE_DIR ${ECHOBOT_EXTERNAL_BUILD_DIR}/romocc/include)
    set(ROMOCC_LIBRARY_DIR ${ECHOBOT_EXTERNAL_BUILD_DIR}/romocc/lib)
    link_directories(${ROMOCC_LIBRARY_DIR})
endif()

list(APPEND ECHOBOT_INCLUDE_DIRS /home/androst/dev/SINTEF/EchoBot/build/external/romocc/src/romocc/source/)
list(APPEND ECHOBOT_INCLUDE_DIRS /home/androst/dev/SINTEF/EchoBot/build/external/romocc/include/)
list(APPEND ECHOBOT_INCLUDE_DIRS /home/androst/dev/SINTEF/EchoBot/build/external/romocc)
list(APPEND LIBRARIES ${ROMOCC_LIBRARY})
list(APPEND ECHOBOT_EXTERNAL_DEPENDENCIES romocc)