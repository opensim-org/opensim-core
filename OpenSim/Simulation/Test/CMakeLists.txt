
file(GLOB TEST_PROGS "test*.cpp")
file(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.obj *.vtp *.stl)

# We'll add tests for metabolics separately, since they depend on an additional
# header.
file(GLOB METABOLICS_PROGS "testMuscleMetabolics*.cpp")
list(REMOVE_ITEM TEST_PROGS ${METABOLICS_PROGS})

OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    DATAFILES ${TEST_FILES}
    LINKLIBS osimCommon osimSimulation osimAnalyses osimActuators osimLepton
    )

OpenSimAddTests(
    TESTPROGRAMS ${METABOLICS_PROGS}
    SOURCES "MuscleMetabolicsHelper.h"
    LINKLIBS osimCommon osimSimulation osimAnalyses osimActuators osimLepton
    )

OpenSimCopySharedTestFiles(std_subject01_walk1_states.sto)
