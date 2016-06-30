add_executable(exampleStretchController
    exampleStretchController.cpp
    helperMethods.h
    StretchController.h
    Spindle.h)
target_link_libraries(exampleStretchController osimTools)
set_target_properties(exampleStretchController PROPERTIES
    PROJECT_LABEL "Testbed Stretch Controller"
    FOLDER "Examples"
    EXCLUDE_FROM_ALL TRUE
    )
