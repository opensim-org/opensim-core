
set(EXAMPLE_BALANCE2D_FILES
        gait2d.osim
        )

OpenSimAddExampleCXX(NAME ExampleStandingBalance2D SUBDIR TaskSpace
    EXECUTABLES ExampleStandingBalance2D
    RESOURCES "${EXAMPLE_BALANCE2D_FILES}")
