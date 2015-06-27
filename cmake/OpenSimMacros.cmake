include(CMakeParseArguments)

# Create an OpenSim API library. Here are the arguments:
# VENDORLIB: If this is a vendor library, specify "VENDORLIB" as the first
#   argument. Otherwise, omit.
# LOWERINCLUDEDIRNAME: When installing the headers for this library, make the
#   name of the library all lower-case (e.g., Lepton -> lepton).
# KIT: Name of the library (e.g., Common).
# AUTHORS: A string listing authors of the library.
# LINKLIBS: List of libraries (targets) to link against.
# INCLUDES: List of header files for the library (obtain via file(GLOB ...)).
# SOURCES: List of cpp files for the library (obtain via file(GLOB ...)).
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
#   opensim_add_library(
#       KIT Common
#       AUTHORS "Clay_Anderson-Ayman_Habib_and_Peter_Loan"
#       LINKLIBS ${Simbody_LIBRARIES}
#       INCLUDES ${INCLUDES}
#       SOURCES ${SOURCES}
#       TESTDIRS "Test"
#       )
function(OPENSIM_ADD_LIBRARY)

    # Parse arguments.
    # ----------------
    # http://www.cmake.org/cmake/help/v2.8.9/cmake.html#module:CMakeParseArguments
    set(options VENDORLIB LOWERINCLUDEDIRNAME)
    set(oneValueArgs KIT AUTHORS)
    set(multiValueArgs LINKLIBS INCLUDES SOURCES TESTDIRS INCLUDEDIRS)
    CMAKE_PARSE_ARGUMENTS(
        OSIMADDLIB "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    string(TOUPPER "${OSIMADDLIB_KIT}" OSIMADDLIB_UKIT)


    # Version stuff.
    # --------------
    set(OSIMADDLIB_LIBRARY_NAME osim${OSIMADDLIB_KIT})

    add_definitions(
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
    include_directories(${OpenSim_SOURCE_DIR})

    # Create the library using the provided source and include files.
    add_library(${OSIMADDLIB_LIBRARY_NAME} SHARED
        ${OSIMADDLIB_SOURCES} ${OSIMADDLIB_INCLUDES})

    # This target links to the libraries provided as arguments to this func.
    target_link_libraries(${OSIMADDLIB_LIBRARY_NAME} ${OSIMADDLIB_LINKLIBS})

    # This is for exporting classes on Windows.
    if(OSIMADDLIB_VENDORLIB)
        set(OSIMADDLIB_PROJECT_LABEL
            "Vendor Libraries - ${OSIMADDLIB_LIBRARY_NAME}")
    else()
        set(OSIMADDLIB_PROJECT_LABEL "Libraries - ${OSIMADDLIB_LIBRARY_NAME}")
    endif()
    set_target_properties(${OSIMADDLIB_LIBRARY_NAME} PROPERTIES
       DEFINE_SYMBOL OSIM${OSIMADDLIB_UKIT}_EXPORTS
       PROJECT_LABEL "${OSIMADDLIB_PROJECT_LABEL}"
    )


    # Install.
    # --------
    # Shared libraries are needed at runtime for applications, so we put them
    # at the top level in OpenSim/bin/*.dll (Windows) or OpenSim/lib/*.so
    # (Linux) or OpemSim/lib/*.dylib (Mac). Windows .lib files, and Linux/Mac
    # .a static archives are only needed at link time so go in sdk/lib.
    if(WIN32)
        set(OSIMADDLIB_LIBRARY_DESTINATION sdk/lib)
    else()
        set(OSIMADDLIB_LIBRARY_DESTINATION lib)
    endif()
    install(TARGETS ${OSIMADDLIB_LIBRARY_NAME}
        EXPORT OpenSimTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION "${OSIMADDLIB_LIBRARY_DESTINATION}"
        ARCHIVE DESTINATION sdk/lib)

    # Install headers.
    # ----------------
    set(_INCLUDE_PREFIX sdk/include)
    if(OSIMADDLIB_VENDORLIB)
        set(_INCLUDE_PREFIX ${_INCLUDE_PREFIX}/Vendors)
    else()
        set(_INCLUDE_PREFIX ${_INCLUDE_PREFIX}/OpenSim)
    endif()
    if(OSIMADDLIB_LOWERINCLUDEDIRNAME)
        string(TOLOWER "${OSIMADDLIB_KIT}" OSIMADDLIB_LKIT)
        set(_INCLUDE_LIBNAME ${OSIMADDLIB_LKIT})
    else()
        set(_INCLUDE_LIBNAME ${OSIMADDLIB_KIT})
    endif()
    if(OSIMADDLIB_INCLUDEDIRS)
        foreach(dir ${OSIMADDLIB_INCLUDEDIRS})
            file(GLOB HEADERS ${dir}/*.h) # returns full pathnames
            install(FILES ${HEADERS}
                DESTINATION ${_INCLUDE_PREFIX}/${_INCLUDE_LIBNAME}/${dir})
        endforeach(dir)
    else()
        install(FILES ${OSIMADDLIB_INCLUDES}
            DESTINATION ${_INCLUDE_PREFIX}/${_INCLUDE_LIBNAME})
    endif()


    # Testing.
    # --------
    enable_testing()

    if(EXECUTABLE_OUTPUT_PATH)
        set(TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
    else()
        set(TEST_PATH .)
    endif()

    if(BUILD_TESTING)
        foreach(OSIMADDLIB_TESTDIR ${OSIMADDLIB_TESTDIRS})
            subdirs("${OSIMADDLIB_TESTDIR}")
        endforeach()
    endif()

endfunction()
