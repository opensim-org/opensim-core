SET(KIT Simulation)
SET(UKIT SIMULATION)

##
## Version stuff
##
SET(OPENSIM_SIMULATION_LIBRARY_NAME osim${KIT})
SET(OPENSIM_SIMULATION_MAJOR_VERSION 2)
SET(OPENSIM_SIMULATION_MINOR_VERSION 2)
SET(OPENSIM_SIMULATION_BUILD_VERSION 0)
SET(OPENSIM_SIMULATION_COPYRIGHT_YEARS "2005-6")
# comma separated list of authors
SET(OPENSIM_SIMULATION_AUTHORS  "Clay_Anderson-Ayman_Habib_and_Peter_Loan") 

ADD_DEFINITIONS(-DOPENSIM_SIMULATION_LIBRARY_NAME=${OPENSIM_SIMULATION_LIBRARY_NAME}
-DOPENSIM_SIMULATION_MAJOR_VERSION=${OPENSIM_SIMULATION_MAJOR_VERSION}
-DOPENSIM_SIMULATION_MINOR_VERSION=${OPENSIM_SIMULATION_MINOR_VERSION}
-DOPENSIM_SIMULATION_BUILD_VERSION=${OPENSIM_SIMULATION_BUILD_VERSION}
-DOPENSIM_SIMULATION_COPYRIGHT_YEARS="${OPENSIM_SIMULATION_COPYRIGHT_YEARS}"
-DOPENSIM_SIMULATION_AUTHORS="${OPENSIM_SIMULATION_AUTHORS}"
-DOPENSIM_SIMULATION_TYPE="shared")

## Name headers under individual directories so that INSTALL doesn't barf on them

FILE(GLOB SOURCE_FILES Control/*.cpp Integrator/*.cpp Manager/*.cpp Model/*.cpp Wrap/*.cpp SimbodyEngine/*.cpp *.cpp)
FILE(GLOB INCLUDE_FILES Control/*.h Integrator/*.h Manager/*.h Model/*.h Wrap/*.h SimbodyEngine/*.h *.h)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors
			 ${XERCES_INCLUDE_DIR})

SET(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

ADD_LIBRARY(osim${KIT} SHARED ${SOURCE_FILES} ${INCLUDE_FILES}) 

IF(WIN32)
TARGET_LINK_LIBRARIES(osim${KIT} 
               debug osimCommon_d optimized osimCommon pthreadVC2		     
		       ${SIMTK_ALL_LIBS}
		       debug xerces-c_2D optimized  xerces-c_2)
ELSE(WIN32)
TARGET_LINK_LIBRARIES(osim${KIT} 
               debug osimCommon_d     optimized osimCommon		     
		       ${SIMTK_ALL_LIBS}
  		       debug xerces-c      optimized xerces-c
		dl)
ENDIF(WIN32)


SET_TARGET_PROPERTIES(osim${KIT} PROPERTIES
   DEFINE_SYMBOL ${EXPORT_MACRO}
   PROJECT_LABEL "Libraries - osim${KIT}"
)

## install stuff
IF (WIN32)
	INSTALL_TARGETS(/sdk/lib/ RUNTIME_DIRECTORY /bin osim${KIT})
ELSE(WIN32)
	INSTALL_TARGETS(/bin/ RUNTIME_DIRECTORY /bin osim${KIT})
ENDIF(WIN32)

SET(SIMULATION_SUBDIRS 
.
Control
Manager
Model
Wrap
SimbodyEngine
)

FOREACH(dir ${SIMULATION_SUBDIRS})
   FILE(GLOB HEADERS ${dir}/*.h)	# returns full pathnames
   INSTALL_FILES(/sdk/include/OpenSim/Simulation/${dir} .h ${HEADERS})
ENDFOREACH(dir)

SUBDIRS(SimbodyEngine Test)
