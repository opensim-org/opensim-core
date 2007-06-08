FILE(GLOB SOURCE_FILES *.cpp)
FILE(GLOB INCLUDE_FILES *.h)

LINK_LIBRARIES(debug osimTools_d optimized osimTools)

ADD_EXECUTABLE(testMultipleSdfastModels ${SOURCE_FILES} ${INCLUDE_FILES})

SET_TARGET_PROPERTIES(testMultipleSdfastModels PROPERTIES
	${EXCLUDE_IF_MINIMAL_BUILD}
	PROJECT_LABEL "Tests - testMultipleSdfastModels"
)
