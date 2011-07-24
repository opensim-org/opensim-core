INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors 
		    ${XERCES_INCLUDE_DIR}
		    ${OpenSim_SOURCE_DIR}/Tools)

ADD_LIBRARY(auxiliaryTestFunctions auxiliaryTestFunctions.cpp)

TARGET_LINK_LIBRARIES(auxiliaryTestFunctions	debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
						debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
						debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
   						debug ${NameSpace}SimTKcommon${CMAKE_DEBUG_POSTFIX} optimized   ${NameSpace}SimTKcommon
   						debug ${NameSpace}SimTKmath${CMAKE_DEBUG_POSTFIX} optimized  ${NameSpace}SimTKmath)

SET_TARGET_PROPERTIES(auxiliaryTestFunctions PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - auxiliaryFunctions")