include_directories(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors ${XERCES_INCLUDE_DIR})

file(GLOB SOURCE_FILES *.cpp)
file(GLOB INCLUDE_FILES *.h)

if(BUILD_STATIC_OSIM_LIBS)
    add_definitions(-DSTATIC_OSIM_LIBS)
endif(BUILD_STATIC_OSIM_LIBS)

link_libraries(debug xerces-c_2D optimized xerces-c_2 
    debug osimTools_d optimized osimTools
    debug osimSimbodyEngine_d optimized osimSimbodyEngine)

add_executable(convertFiles ${SOURCE_FILES} ${INCLUDE_FILES})

set_target_properties(convertFiles PROPERTIES
    ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Utilities - convertFiles")

install(TARGETS convertFiles DESTINATION "${CMAKE_INSTALL_BINDIR}")
