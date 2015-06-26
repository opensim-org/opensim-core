cmake_minimum_required(VERSION 2.6)

include_directories(${OpenSim_SOURCE_DIR} 
            ${OpenSim_SOURCE_DIR}/Vendors)

if(WIN32)
  set( THREAD_LIB pthreadVC2)
else(WIN32)
  set( THREAD_LIB )
endif(WIN32)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX ${OPENSIM_INSTALL_DIR})
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

if(BUILD_TESTING)

add_subdirectory(Components)
add_subdirectory(ControllerExample)
add_subdirectory(CoupledBushingForceExample)
add_subdirectory(ExampleMain)
add_subdirectory(OptimizationExample_Arm26)
add_subdirectory(CustomActuatorExample)
add_subdirectory(MuscleExample)
add_subdirectory(SimpleOptimizationExample)
add_subdirectory(Environment)
add_subdirectory(testIterators)
add_subdirectory(README)

add_subdirectory(Wrapping)
endif(BUILD_TESTING)

