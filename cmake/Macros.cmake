#### Macro for adding source files and directories
macro (project_add_sources)
  file (RELATIVE_PATH _relPath "${PROJECT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
  foreach (_src ${ARGN})
    if (_relPath)
      list (APPEND ECHOBOT_SOURCE_FILES "${_relPath}/${_src}")
    else()
      list (APPEND ECHOBOT_SOURCE_FILES "${_src}")
    endif()
  endforeach()
  if (_relPath)
    # propagate CORAH_SOURCE_FILES to parent directory
    set (ECHOBOT_SOURCE_FILES ${ECHOBOT_SOURCE_FILES} PARENT_SCOPE)
  endif()
endmacro()

#### Macro for adding subdirectories
macro (project_add_subdirectories)
    file (RELATIVE_PATH _relPath "${PROJECT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    foreach (_src ${ARGN})
        add_subdirectory(${_src})
    endforeach()
    if (_relPath)
        # propagate to parent directory
        set (ECHOBOT_SOURCE_FILES ${ECHOBOT_SOURCE_FILES} PARENT_SCOPE)
    endif()
endmacro()

### Macro for add application
macro (project_add_application NAME)
    list(APPEND ECHOBOT_APPS ${NAME})
    add_executable(${NAME} ${ARGN})
    target_link_libraries(${NAME} EchoBot)
    install(TARGETS ${NAME}
            DESTINATION echobot/bin
            )
    file (RELATIVE_PATH _relPath "${PROJECT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    if(_relPath)
        # propagate to parent directory
        set(ECHOBOT_APPS ${ECHOBOT_APPS} PARENT_SCOPE)
    endif()
endmacro()