find_package(Catch2 REQUIRED HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

OpenSimCopySharedTestFiles(gait10dof18musc_subject01.osim)

OpenSimAddTest(NAME testComponents
        LINKLIBS osimTools)
