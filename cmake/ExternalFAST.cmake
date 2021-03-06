# Download and set up Eigen

include(cmake/Externals.cmake)

ExternalProject_Add(FAST
        PREFIX ${ECHOBOT_EXTERNAL_BUILD_DIR}/FAST
        BINARY_DIR ${ECHOBOT_EXTERNAL_BUILD_DIR}/FAST
        GIT_REPOSITORY "https://github.com/androst/FAST.git"
        GIT_TAG "echobot"
        CMAKE_ARGS
            -DFAST_MODULE_OpenIGTLink=ON
            -DFAST_MODULE_RealSense=ON
            -DFAST_MODULE_Visualization=ON
            -DFAST_MODULE_TensorFlow=ON
            -DFAST_BUILD_TensorFlow_CUDA=ON
            -DFAST_MODULE_WholeSlideImaging=ON
            -DFAST_MODULE_Clarius=ON
            -DFAST_BUILD_TOOLS=OFF
            -DCLARIUS_SDK_DIR=${CLARIUS_SDK_DIR}
        CMAKE_CACHE_ARGS
            -DOpenCL_INCLUDE_DIR:PATH=${OpenCL_INCLUDE_DIR}
            -DOpenCL_LIBRARY:PATH=${OpenCL_LIBRARY}
            -DCMAKE_INSTALL_PREFIX:PATH=${ECHOBOT_EXTERNAL_BUILD_DIR}
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DCMAKE_VERBOSE_MAKEFILE:BOOL=OFF
            -DCMAKE_INSTALL_MESSAGE:BOOL=LAZY
)

if(WIN32)
    set(FAST_LIBRARY FAST.lib)
else()
    set(FAST_LIBRARY ${CMAKE_SHARED_LIBRARY_PREFIX}FAST${CMAKE_SHARED_LIBRARY_SUFFIX})
    set(FAST_INCLUDE_DIR ${ECHOBOT_EXTERNAL_BUILD_DIR}/FAST/fast/include)
    set(FAST_LIBRARY_DIR ${ECHOBOT_EXTERNAL_BUILD_DIR}/FAST/lib)
    link_directories(${FAST_LIBRARY_DIR})
endif()

list(APPEND ECHOBOT_INCLUDE_DIRS ${FAST_INCLUDE_DIR})
list(APPEND ECHOBOT_INCLUDE_DIRS ${OpenCL_INCLUDE_DIR})
list(APPEND LIBRARIES ${FAST_LIBRARY} pthread)
list(APPEND ECHOBOT_EXTERNAL_DEPENDENCIES FAST)