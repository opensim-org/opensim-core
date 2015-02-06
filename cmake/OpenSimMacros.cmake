include(CMakeParseArguments)

# Create an OpenSim API library. Here are the arguments:
# VENDORLIB: If this is a vendor library, specify "VENDORLIB" as the first
#   argument. Otherwise, omit.
# LOWERINCLUDEDIRNAME: When installing the headers for this library, make the
#   name of the library all lower-case (e.g., Lepton -> lepton).
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
#       LINKLIBS ${Simbody_LIBRARIES}
#       INCLUDES ${INCLUDES}
#       SOURCES ${SOURCES}
#       TESTDIRS "Test"
#       )
FUNCTION(OPENSIM_ADD_LIBRARY)

    # Parse arguments.
    # ----------------
    # http://www.cmake.org/cmake/help/v2.8.9/cmake.html#module:CMakeParseArguments
    SET(options VENDORLIB LOWERINCLUDEDIRNAME)
    SET(oneValueArgs KIT AUTHORS)
    SET(multiValueArgs LINKLIBS INCLUDES SOURCES TESTDIRS INCLUDEDIRS)
    CMAKE_PARSE_ARGUMENTS(
        OSIMADDLIB "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    STRING(TOUPPER "${OSIMADDLIB_KIT}" OSIMADDLIB_UKIT)


    # Version stuff.
    # --------------
    SET(OSIMADDLIB_LIBRARY_NAME osim${OSIMADDLIB_KIT})

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
    INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR})

    # Create the library using the provided source and include files.
    ADD_LIBRARY(${OSIMADDLIB_LIBRARY_NAME} SHARED
        ${OSIMADDLIB_SOURCES} ${OSIMADDLIB_INCLUDES})

    # This target links to the libraries provided as arguments to this func.
    TARGET_LINK_LIBRARIES(${OSIMADDLIB_LIBRARY_NAME} ${OSIMADDLIB_LINKLIBS})

    # This is for exporting classes on Windows.
    IF(OSIMADDLIB_VENDORLIB)
        SET(OSIMADDLIB_PROJECT_LABEL
            "Vendor Libraries - ${OSIMADDLIB_LIBRARY_NAME}")
    ELSE()
        SET(OSIMADDLIB_PROJECT_LABEL "Libraries - ${OSIMADDLIB_LIBRARY_NAME}")
    ENDIF()
    SET_TARGET_PROPERTIES(${OSIMADDLIB_LIBRARY_NAME} PROPERTIES
       DEFINE_SYMBOL OSIM${OSIMADDLIB_UKIT}_EXPORTS
       PROJECT_LABEL "${OSIMADDLIB_PROJECT_LABEL}"
    )


    # Install.
    # --------
    # Shared libraries are needed at runtime for applications, so we put them
    # at the top level in OpenSim/bin/*.dll (Windows) or OpenSim/lib/*.so
    # (Linux) or OpemSim/lib/*.dylib (Mac). Windows .lib files, and Linux/Mac
    # .a static archives are only needed at link time so go in sdk/lib.
    IF(WIN32)
        SET(OSIMADDLIB_LIBRARY_DESTINATION sdk/lib)
    ELSE()
        SET(OSIMADDLIB_LIBRARY_DESTINATION lib)
    ENDIF()
    INSTALL(TARGETS ${OSIMADDLIB_LIBRARY_NAME}
        EXPORT OpenSimTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION "${OSIMADDLIB_LIBRARY_DESTINATION}"
        ARCHIVE DESTINATION sdk/lib)

    # Install headers.
    # ----------------
    SET(_INCLUDE_PREFIX sdk/include)
    IF(OSIMADDLIB_VENDORLIB)
        SET(_INCLUDE_PREFIX ${_INCLUDE_PREFIX}/Vendors)
    ELSE()
        SET(_INCLUDE_PREFIX ${_INCLUDE_PREFIX}/OpenSim)
    ENDIF()
    IF(OSIMADDLIB_LOWERINCLUDEDIRNAME)
        STRING(TOLOWER "${OSIMADDLIB_KIT}" OSIMADDLIB_LKIT)
        SET(_INCLUDE_LIBNAME ${OSIMADDLIB_LKIT})
    ELSE()
        SET(_INCLUDE_LIBNAME ${OSIMADDLIB_KIT})
    ENDIF()
    IF(OSIMADDLIB_INCLUDEDIRS)
        FOREACH(dir ${OSIMADDLIB_INCLUDEDIRS})
            FILE(GLOB HEADERS ${dir}/*.h) # returns full pathnames
            INSTALL(FILES ${HEADERS}
                DESTINATION ${_INCLUDE_PREFIX}/${_INCLUDE_LIBNAME}/${dir})
        ENDFOREACH(dir)
    ELSE()
        INSTALL(FILES ${OSIMADDLIB_INCLUDES}
            DESTINATION ${_INCLUDE_PREFIX}/${_INCLUDE_LIBNAME})
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
