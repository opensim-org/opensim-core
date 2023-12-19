file(GLOB SOURCE_FILES src/*.cpp)

if(WIN32)
    add_definitions("-DLEPTON_BUILDING_SHARED_LIBRARY")
endif(WIN32)

OpenSimAddLibrary(VENDORLIB LOWERINCLUDEDIRNAME
    KIT Lepton
    AUTHORS "Peter_Eastman"
    LINKLIBS ""
    INCLUDES "include/Lepton.h"
    SOURCES ${SOURCE_FILES}
    TESTDIRS test
    INCLUDEDIRS include include/lepton
    INCLUDEINSTALLREL include/lepton
    )

if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    # Avoid clang's warning:
    # ExpressionTreeNode.cpp:62:90: Reference cannot be bound to dereferenced
    # null pointer in well-defined C++ code; comparison may be assumed to
    # always evaluate to false.
    set_source_files_properties("src/ExpressionTreeNode.cpp"
        PROPERTIES COMPILE_FLAGS "-Wno-tautological-undefined-compare")
    # This warning also occurs in ParsedExpression.cpp:49.
    set_source_files_properties("src/ParsedExpression.cpp"
        PROPERTIES COMPILE_FLAGS "-Wno-tautological-undefined-compare")
    # We don't want to edit the lepton files (that would cause them to diverge
    # from the copy in OpenMM), so we ignore these warnings via CMake settings.
endif()
