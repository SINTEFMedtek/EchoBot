# Download and set up Eigen

include(cmake/Externals.cmake)

ExternalProject_Add(FAST
        PREFIX ${ECHOBOT_EXTERNAL_BUILD_DIR}/FAST
        BINARY_DIR ${ECHOBOT_EXTERNAL_BUILD_DIR}/FAST
        GIT_REPOSITORY "https://github.com/smistad/FAST.git"
        GIT_TAG "ca95829b949d72d5a5cc34bd4e58deefc10097b6"
        CMAKE_CACHE_ARGS
            -DCMAKE_PREFIX_PATH:PATH=${ECHOBOT_EXTERNAL_INSTALL_DIR}/lib/cmake/Qt5
            -DOpenCL_INCLUDE_DIR:PATH=${OpenCL_INCLUDE_DIR}
            -DOpenCL_LIBRARY:PATH=${OpenCL_LIBRARY}
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DCMAKE_VERBOSE_MAKEFILE:BOOL=OFF
            -DCMAKE_INSTALL_MESSAGE:BOOL=LAZY
            -DBUILD_TESTING:BOOL=OFF
            -DFAST_BUILD_TESTS:BOOL=ON
            -DFAST_BUILD_EXAMPLES:BOOL=ON
            -DFAST_BUILD_TOOLS:BOOL=ON
            -DCMAKE_INSTALL_PREFIX:PATH=${ECHOBOT_EXTERNAL_BUILD_DIR}/FAST
            -DFAST_MODULE_Visualization:BOOL=ON
            -DFAST_MODULE_OpenIGTLink:BOOL=ON
            -DFAST_MODULE_RealSense:BOOL=ON
            -DFAST_MODULE_TensorFlow:BOOL=OFF
            -DFAST_BUILD_QT5:BOOL=OFF
)
ExternalProject_Get_Property(FAST install_dir)

if(WIN32)
    set(FAST_LIBRARY corah.lib)
else()
    set(FAST_LIBRARY ${CMAKE_SHARED_LIBRARY_PREFIX}FAST${CMAKE_SHARED_LIBRARY_SUFFIX})
endif()

list(APPEND ECHOBOT_INCLUDE_DIRS ${install_dir}/fast/include)
list(APPEND LIBRARIES ${install_dir}/lib/${FAST_LIBRARY})
list(APPEND ECHOBOT_EXTERNAL_DEPENDENCIES FAST)