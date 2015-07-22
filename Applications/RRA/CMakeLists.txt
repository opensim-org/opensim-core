set(APPNAME rra)

include_directories(${OpenSim_SOURCE_DIR}
                          ${OpenSim_SOURCE_DIR}/Vendors)

# cmake should automatically figure out what other libraries are needed
link_libraries(debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
               debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
               debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation)

add_executable(${APPNAME} ${APPNAME}.cpp)
install_targets(/bin/ ${APPNAME})
set_target_properties(${APPNAME} PROPERTIES PROJECT_LABEL "Applications - ${APPNAME}")

#
# Testing
#

#if(UNIX)
#  add_definitions(-fprofile-arcs -ftest-coverage)
#  link_libraries(gcov)
#endif(UNIX)

if(BUILD_TESTING)
    subdirs(test)
endif(BUILD_TESTING)


