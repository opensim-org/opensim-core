SET(KIT Simulation)
SET(UKIT SIMULATION)

SET(SIMULATION_SUBDIRS 
        .
        Control
        Manager
        Model
        Wrap
        SimbodyEngine)


##
## Version stuff
##
SET(OPENSIM_SIMULATION_LIBRARY_NAME osim${KIT})
SET(OPENSIM_SIMULATION_MAJOR_VERSION 3)
SET(OPENSIM_SIMULATION_MINOR_VERSION 0)
SET(OPENSIM_SIMULATION_BUILD_VERSION 0)
SET(OPENSIM_SIMULATION_COPYRIGHT_YEARS "2005-13")
# comma separated list of authors
SET(OPENSIM_SIMULATION_AUTHORS  "Ajay_Seth-Michael_Sherman_and_Ayman_Habib") 

ADD_DEFINITIONS(-DOPENSIM_SIMULATION_LIBRARY_NAME=${OPENSIM_SIMULATION_LIBRARY_NAME}
-DOPENSIM_SIMULATION_MAJOR_VERSION=${OPENSIM_SIMULATION_MAJOR_VERSION}
-DOPENSIM_SIMULATION_MINOR_VERSION=${OPENSIM_SIMULATION_MINOR_VERSION}
-DOPENSIM_SIMULATION_BUILD_VERSION=${OPENSIM_SIMULATION_BUILD_VERSION}
-DOPENSIM_SIMULATION_COPYRIGHT_YEARS="${OPENSIM_SIMULATION_COPYRIGHT_YEARS}"
-DOPENSIM_SIMULATION_AUTHORS="${OPENSIM_SIMULATION_AUTHORS}"
-DOPENSIM_SIMULATION_TYPE="shared")

## Name headers under individual directories so that INSTALL doesn't barf on them

SET(SOURCE_FILES)
SET(INCLUDE_FILES)
FOREACH(dir ${SIMULATION_SUBDIRS})
    FILE(GLOB SOURCE_FILES_ONEDIR ${dir}/*.cpp)
    FILE(GLOB INCLUDE_FILES_ONEDIR ${dir}/*.h)
    LIST(APPEND SOURCE_FILES ${SOURCE_FILES_ONEDIR})
    LIST(APPEND INCLUDE_FILES ${INCLUDE_FILES_ONEDIR})
ENDFOREACH()

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
            ${OpenSim_SOURCE_DIR}/Vendors)

SET(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

ADD_LIBRARY(osim${KIT} SHARED ${SOURCE_FILES} ${INCLUDE_FILES}) 

IF(WIN32)
TARGET_LINK_LIBRARIES(osim${KIT} 
               debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
               debug osimLepton${CMAKE_DEBUG_POSTFIX} optimized osimLepton
               ${SIMTK_ALL_LIBS})
ELSE(WIN32)
TARGET_LINK_LIBRARIES(osim${KIT} 
               debug osimCommon${CMAKE_DEBUG_POSTFIX}     optimized osimCommon
               debug osimLepton${CMAKE_DEBUG_POSTFIX}	  optimized osimLepton
               ${SIMTK_ALL_LIBS}
        dl)
ENDIF(WIN32)


SET_TARGET_PROPERTIES(osim${KIT} PROPERTIES
   DEFINE_SYMBOL ${EXPORT_MACRO}
   PROJECT_LABEL "Libraries - osim${KIT}"
)

# Shared libraries are needed at runtime for applications, so we put them
# at the top level in OpenSim/bin/*.dll (Windows) or OpenSim/lib/*.so (Linux)
# or OpemSim/lib/*.dylib (Mac). Windows .lib files, and Linux/Mac .a static
# archives are only needed at link time so go in sdk/lib.
IF(WIN32)
    INSTALL(TARGETS osim${KIT}
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION sdk/lib
            ARCHIVE DESTINATION sdk/lib)
ELSE()
    INSTALL(TARGETS osim${KIT}
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION sdk/lib)
ENDIF()

FOREACH(dir ${SIMULATION_SUBDIRS})
   FILE(GLOB HEADERS ${dir}/*.h)	# returns full pathnames
   INSTALL(FILES ${HEADERS}
        DESTINATION sdk/include/OpenSim/${KIT}/${dir})
ENDFOREACH(dir)

IF(BUILD_TESTING)
    # SimbodyEngine has a Test subdirectory also
    SUBDIRS(SimbodyEngine Test)
ENDIF()
