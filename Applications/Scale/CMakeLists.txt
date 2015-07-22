set(APPNAME scale)

include_directories(${OpenSim_SOURCE_DIR}
                          ${OpenSim_SOURCE_DIR}/Vendors)

add_executable(${APPNAME} ${APPNAME}.cpp)

# cmake should automatically figure out what other libraries are needed
target_link_libraries(${APPNAME}
          debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
          debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
          debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
          )


install_targets(/bin/ ${APPNAME})
set_target_properties(${APPNAME} PROPERTIES PROJECT_LABEL "Applications - ${APPNAME}")

#
# Testing
#

#if(UNIX)
#  add_definitions(-fprofile-arcs -ftest-coverage)
#  link_libraries(gcov)
#endif(UNIX)

#enable_testing()

if(EXECUTABLE_OUTPUT_PATH)
  set(TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
else(EXECUTABLE_OUTPUT_PATH)
  set(TEST_PATH .)
endif(EXECUTABLE_OUTPUT_PATH)

if(BUILD_TESTING)
    subdirs(test)
endif(BUILD_TESTING)
