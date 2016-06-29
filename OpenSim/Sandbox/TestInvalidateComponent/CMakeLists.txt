add_executable(testInvalidateComponent
    exampleStretchController.cpp
    helperMethods.h
    StretchController.h)
target_link_libraries(testInvalidateComponent osimTools)
set_target_properties(testInvalidateComponent PROPERTIES
    PROJECT_LABEL "Test Invalidate Component"
    FOLDER "Examples"
    EXCLUDE_FROM_ALL TRUE
    )
