# Settings.
# ---------
set(CMD_NAME "joint-mechanics")

# Configure this project.
# -----------------------
file(GLOB SOURCE_FILES *.h *.cpp *.c)

add_executable(${CMD_NAME} ${SOURCE_FILES})

target_link_libraries(${CMD_NAME} ${OpenSim_LIBRARIES})
target_link_libraries(${CMD_NAME} ${PLUGIN_NAME})

SET_TARGET_PROPERTIES (${CMD_NAME} PROPERTIES FOLDER cmd_tools)

install(TARGETS ${CMD_NAME} DESTINATION cmd_tools)
