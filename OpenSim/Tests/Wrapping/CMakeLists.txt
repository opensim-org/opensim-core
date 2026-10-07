find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")


OpenSimAddTest(NAME testGeometryPathWrapping
        LINKLIBS osimTools
        RESOURCES std_testGeometryPathWrapping_subject_walk_armless_muscle_lengths.sto
                  std_testGeometryPathWrapping_walk_gait1018_muscle_lengths.sto
                  testGeometryPathWrapping_ShoulderWrappingWithPathSprings.osim
        )

OpenSimAddTest(NAME testScholz2015GeometryPath
        LINKLIBS osimTools)

OpenSimAddTest(NAME testWrapCylinder
        LINKLIBS osimTools)

OpenSimAddTest(NAME testWrapObject
        LINKLIBS osimTools
        RESOURCES testWrapObject_updateFromXMLNode30515.osim
        )
