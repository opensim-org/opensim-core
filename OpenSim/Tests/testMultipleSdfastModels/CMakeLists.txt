FILE(GLOB SOURCE_FILES *.cpp)
FILE(GLOB INCLUDE_FILES *.h)

LINK_LIBRARIES(debug osimTools_d optimized osimTools)

ADD_EXECUTABLE(testMultipleSdfastModels ${SOURCE_FILES} ${INCLUDE_FILES})

#INSTALL_TARGETS(/bin/ testMultipleSdfastModels)
