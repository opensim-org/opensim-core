include(CMakeParseArguments)

# Create an OpenSim API library. Here are the arguments:
# KIT: Name of the library (e.g., Common).
# AUTHORS: A string listing authors of the library.
# LINKLIBS: List of libraries (targets) to link against.
# INCLUDES: List of header files for the library (obtain via FILE(GLOB ...)).
# SOURCES: List of cpp files for the library (obtain via FILE(GLOB ...)).
# TESTDIRS: List of subdirectories that contain tests (and a CMakeLists.txt).
# INCLUDEDIRS (optional): Affects how header files are installed. Use this if
#   the library directory contains subdirectories with header files. If this is
#   the case, this variable should be a list of those subdirectories (relative
#   paths). See OpenSim/Simulation/CMakeLists.txt for an example. If omitted,
#   all the headers specified under INCLUDES are installed into the same
#   directory in the installation tree.
#
# Here's an example from OpenSim/Common/CMakeLists.txt:
#
#   OPENSIM_ADD_LIBRARY(
#       KIT Common
#       AUTHORS "Clay_Anderson-Ayman_Habib_and_Peter_Loan"
#       LINKLIBS ${SIMTK_COMMON_LIB} ${SIMTK_MATH_LIB} ${MATH_LIBS_TO_USE}
#       INCLUDES ${INCLUDES}
#       SOURCES ${SOURCES}
#       TESTDIRS "Test"
#       )
FUNCTION(OPENSIM_ADD_LIBRARY)

    # Parse arguments.
    # ----------------
    # http://www.cmake.org/cmake/help/v2.8.9/cmake.html#module:CMakeParseArguments
    SET(options "")
    SET(oneValueArgs KIT AUTHORS)
    SET(multiValueArgs LINKLIBS INCLUDES SOURCES TESTDIRS INCLUDEDIRS)
    CMAKE_PARSE_ARGUMENTS(
        OSIMADDLIB "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    STRING(TOUPPER "${OSIMADDLIB_KIT}" OSIMADDLIB_UKIT)


    # Version stuff.
    # --------------
    SET(OSIMADDLIB_LIBRARY_NAME osim${OSIMADDLIB_KIT})
    SET(COPYRIGHT_YEARS "2005-14")

    ADD_DEFINITIONS(
        -DOPENSIM_${OSIMADDLIB_UKIT}_LIBRARY_NAME=${OSIMADDLIB_LIBRARY_NAME}
        -DOPENSIM_${OSIMADDLIB_UKIT}_MAJOR_VERSION=${OPENSIM_MAJOR_VERSION}
        -DOPENSIM_${OSIMADDLIB_UKIT}_MINOR_VERSION=${OPENSIM_MINOR_VERSION}
        -DOPENSIM_${OSIMADDLIB_UKIT}_BUILD_VERSION=${OPENSIM_PATCH_VERSION}
        -DOPENSIM_${OSIMADDLIB_UKIT}_COPYRIGHT_YEARS="2005-2014"
        -DOPENSIM_${OSIMADDLIB_UKIT}_AUTHORS="${AUTHORS}"
        -DOPENSIM_${OSIMADDLIB_UKIT}_TYPE="Shared"
        )


    # Add the library.
    # ----------------
    # These next few lines are the most important:
    # Specify the directories in OpenSim that contain header files.
    INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR}/Vendors ${OpenSim_SOURCE_DIR})

    # Create the library using the provided source and include files.
    ADD_LIBRARY(osim${OSIMADDLIB_KIT} SHARED
        ${OSIMADDLIB_SOURCES} ${OSIMADDLIB_INCLUDES})

    # This target links to the libraries provided as arguments to this func.
    TARGET_LINK_LIBRARIES(osim${OSIMADDLIB_KIT} ${OSIMADDLIB_LINKLIBS})

    # This is for exporting classes on Windows.
    SET(EXPORT_MACRO OSIM${OSIMADDLIB_UKIT}_EXPORTS)
    SET_TARGET_PROPERTIES(osim${OSIMADDLIB_KIT} PROPERTIES
       DEFINE_SYMBOL ${EXPORT_MACRO}
       PROJECT_LABEL "Libraries - osim${OSIMADDLIB_KIT}"
    )


    # Install.
    # --------
    # Shared libraries are needed at runtime for applications, so we put them
    # at the top level in OpenSim/bin/*.dll (Windows) or OpenSim/lib/*.so
    # (Linux) or OpemSim/lib/*.dylib (Mac). Windows .lib files, and Linux/Mac
    # .a static archives are only needed at link time so go in sdk/lib.
    IF(WIN32)
        SET(OSIMLIB_LIBRARY_DESTINATION sdk/lib)
    ELSE()
        SET(OSIMLIB_LIBRARY_DESTINATION lib)
    ENDIF()
    INSTALL(TARGETS osim${OSIMADDLIB_KIT}
        EXPORT OpenSimTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION "${OSIMLIB_LIBRARY_DESTINATION}"
        ARCHIVE DESTINATION sdk/lib)

    # Install headers.
    # ----------------
    IF(OSIMADDLIB_INCLUDEDIRS)
        FOREACH(dir ${OSIMADDLIB_INCLUDEDIRS})
            FILE(GLOB HEADERS ${dir}/*.h) # returns full pathnames
            INSTALL(FILES ${HEADERS}
                DESTINATION sdk/include/OpenSim/${OSIMADDLIB_KIT}/${dir})
        ENDFOREACH(dir)
    ELSE()
        INSTALL(FILES ${OSIMADDLIB_INCLUDES}
            DESTINATION sdk/include/OpenSim/${OSIMADDLIB_KIT})
    ENDIF()


    # Testing.
    # --------
    ENABLE_TESTING()

    IF(EXECUTABLE_OUTPUT_PATH)
        SET(TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
    ELSE()
        SET(TEST_PATH .)
    ENDIF()

    IF(BUILD_TESTING)
        FOREACH(OSIMADDLIB_TESTDIR ${OSIMADDLIB_TESTDIRS})
            SUBDIRS("${OSIMADDLIB_TESTDIR}")
        ENDFOREACH()
    ENDIF()

ENDFUNCTION()
