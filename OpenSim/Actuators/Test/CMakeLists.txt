
FILE(GLOB TEST_PROGS "test*.cpp")
OPENSIM_ADD_TESTS(
    TESTPROGRAMS ${TEST_PROGS}
    LINKLIBS osimTools
    )

#IF (UNIX)
#  ADD_DEFINITIONS(-fprofile-arcs -ftest-coverage)
#  LINK_LIBRARIES(gcov)
#ENDIF (UNIX)
