#Add all tests
#-------------
FILE(GLOB CMD_NAMES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/*")
FOREACH(cmd_name ${CMD_NAMES})
    IF(IS_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/${cmd_name})
        add_subdirectory(${cmd_name})
    ENDIF()
ENDFOREACH(cmd_name)

#Copy Shared Folder to build
#---------------------------
#file(COPY shared DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
