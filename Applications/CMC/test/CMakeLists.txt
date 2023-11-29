
file(GLOB TEST_PROGS "test*.cpp")
file(GLOB TEST_FILES *.osim *.xml *.sto *.mot)

#OpenSimAddTests(
#    TESTPROGRAMS ${TEST_PROGS}
#    DATAFILES ${TEST_FILES}
#    LINKLIBS osimTools
#    )

OpenSimAddTest(NAME testCMCGait10dof18musc RESOURCES
        gait10dof18musc_Setup_CMC.xml
        gait10dof18musc_std_walk_subject_states_unix.sto
        gait10dof18musc_std_walk_subject_states_win.sto
        gait10dof18musc_subject01.osim
        gait10dof18musc_subject01_walk_grf.mot
        gait10dof18musc_subject01_walk_grf.xml
        gait10dof18musc_subject_adjusted_Kinematics_q.sto
        gait10dof_Kinematics_Tracking_Tasks.xml
        gait10dof_Reserve_Actuators.xml
        LIB_DEPENDS osimTools
        )
