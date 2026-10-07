find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

OpenSimAddTest(NAME testActuators
        LINKLIBS osimTools)

OpenSimAddTest(NAME testDeGrooteFregly2016Muscle
        LINKLIBS osimTools)

OpenSimAddTest(NAME testModelProcessor
        LINKLIBS osimTools)

OpenSimAddTest(NAME testMuscleFirstOrderActivationDynamicModel
        LINKLIBS osimTools)

OpenSimAddTest(NAME testMuscleFixedWidthPennationModel
        LINKLIBS osimTools)

OpenSimAddTest(NAME testMuscles
        LINKLIBS osimTools)

OpenSimAddTest(NAME testPolynomialPathFitter
        LINKLIBS osimTools)

OpenSimAddTest(NAME testSerializableMuscleCurves
        LINKLIBS osimTools)
