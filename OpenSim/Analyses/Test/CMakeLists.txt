find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

file(GLOB TEST_PROGS "test*.cpp")
file(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.obj *.vtp *.stl)

OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    DATAFILES ${TEST_FILES}
    LINKLIBS osimCommon osimSimulation osimAnalyses osimActuators osimLepton Catch2::Catch2WithMain
    )

