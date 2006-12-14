FILE(GLOB SOURCE_FILES *.cpp)
FILE(GLOB INCLUDE_FILES *.h)

## The following lines are commented out temporarily until GUI is functional
## FIND_PACKAGE(VTK)
## IF (VTK_FOUND)
##   INCLUDE(${VTK_USE_FILE})
## ELSE(VTK_FOUND)
##   MESSAGE(FATAL_ERROR
##      "Cannot build without VTK.  Please set VTK_DIR.")
## ENDIF (VTK_FOUND)

INCLUDE_DIRECTORIES(${NMBLApp_SOURCE_DIR} 
					${NMBLApp_SOURCE_DIR}/Vendors 
					${XERCES_INCLUDE_DIR}
					)


LINK_LIBRARIES( debug xerces-c_2D optimized xerces-c_2
				debug rdTools_D optimized rdTools
				debug rdSimulation_D optimized rdSimulation
				debug rdSQP_D optimized rdSQP
				debug rdActuators_D optimized rdActuators
## 				vtkCommon vtkIO
				)
ADD_EXECUTABLE(simmReadXML ${SOURCE_FILES} ${INCLUDE_FILES})
