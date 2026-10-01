find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

OpenSimAddTest(NAME testScale
        LINKLIBS osimTools
        RESOURCES resources/gait2354_Scale_MarkerSet.xml
                  resources/gait2354_simbody.osim
                  resources/std_subject01_scaleSet_applied.xml
                  resources/std_subject01_simbody.osim
                  resources/std_toyLigamentModelScaled.osim
                  resources/subject01_Setup_Scale.xml
                  resources/subject01_Setup_Scale_GUI.xml
                  resources/subject01_static.trc
                  resources/toyLigamentModel.osim
                  resources/toyLigamentModel_Setup_Scale.xml
        )
