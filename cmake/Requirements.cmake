# Setup all dependencies, both internal (have to be installed on the system)
# and external (downloaded and built automatically)



## External depedencies
# FAST
find_package(FAST REQUIRED)
list(APPEND FASTROMO_INCLUDE_DIRS ${FAST_INCLUDE_DIRS})
list(APPEND FASTROMO_SYSTEM_LIBRARIES ${FAST_LIBRARY_DIRS})
#include(cmake/ExternalFAST.cmake)

# Corah
include(cmake/ExternalCorah.cmake)
message("-- Corah: ${CORAH_INCLUDE_DIRS}")
list(APPEND FASTROMO_INCLUDE_DIRS ${CORAH_INCLUDE_DIRS})
list(APPEND FASTROMO_SYSTEM_LIBRARIES ${CORAH_LIBRARY_DIRS})

# Make sure project can find external includes and libaries
link_directories(${FASTROMO_EXTERNAL_INSTALL_DIR}/lib/ ${FASTROMO_SYSTEM_LIBRARIES})
list(APPEND FASTROMO_INCLUDE_DIRS ${FASTROMO_EXTERNAL_INSTALL_DIR}/include)