# Settings.
# ---------
set(CMD_NAME "comak")

# Configure this project.
# -----------------------
file(GLOB SOURCE_FILES *.h *.cpp *.c)

add_executable(${CMD_NAME} ${SOURCE_FILES})

target_link_libraries(${CMD_NAME} ${OpenSim_LIBRARIES})
target_link_libraries(${CMD_NAME} ${PLUGIN_NAME})

SET_TARGET_PROPERTIES (${CMD_NAME} PROPERTIES FOLDER cmd_tools)

#file(COPY inputs DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
#file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/results)

install(TARGETS ${CMD_NAME} DESTINATION cmd_tools)
