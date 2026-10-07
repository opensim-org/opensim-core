find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

OpenSimCopySharedTestFiles(arm26.osim)

OpenSimAddTest(NAME testIterators
        LINKLIBS osimSimulation osimActuators)
