file(GLOB SOURCE_FILES src/*.cpp)

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

if(WIN32)
    # Lepton uses its own build-type symbols (see windowsIncludes.h).
    # Set them based on the actual target type so static and shared builds
    # both work without manual preprocessor flags.
    # PRIVATE: evaluated in this target's context -- TYPE refers to osimLepton.
    # PUBLIC: evaluated in each consumer's context -- use explicit target name
    #         so TYPE still resolves to osimLepton, not the consumer.
    target_compile_definitions(osimLepton
        PRIVATE
            $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:LEPTON_BUILDING_SHARED_LIBRARY>
            $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,STATIC_LIBRARY>:LEPTON_BUILDING_STATIC_LIBRARY>
        PUBLIC
            $<$<STREQUAL:$<TARGET_PROPERTY:osimLepton,TYPE>,STATIC_LIBRARY>:LEPTON_USE_STATIC_LIBRARIES>
    )
endif()

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
