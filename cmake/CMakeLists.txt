# CMake packaging so people can use OpenSim.
# ------------------------------------------

# Requires CMake 2.8.8.
include(CMakePackageConfigHelpers)

set(OPENSIM_INSTALL_CMAKE_DIR cmake)

macro(OpenSimWriteImportedTarget varname targetname)
    get_property(_configs TARGET ${targetname} PROPERTY IMPORTED_CONFIGURATIONS)
    get_property(_type TARGET ${targetname} PROPERTY TYPE)
    if(${_type} STREQUAL "SHARED_LIBRARY")
        set(_static_or_shared "SHARED")
    elseif(${_type} STREQUAL "STATIC_LIBRARY")
        set(_static_or_shared "STATIC")
    else()
        message(AUTHOR_WARNING "Expected target to be shared/static library.")
    endif()

    set(${varname} "${${varname}}\n    add_library(${targetname} ${_static_or_shared} IMPORTED)")
    set(${varname} "${${varname}}\n    set_target_properties(${targetname} PROPERTIES IMPORTED_CONFIGURATIONS \"${_configs}\")")
    get_property(_configs TARGET ${targetname} PROPERTY IMPORTED_CONFIGURATIONS)
    foreach(_cfg ${_configs})
        get_property(_imploc_old TARGET ${targetname} PROPERTY
            IMPORTED_LOCATION_${_cfg})
        string(REPLACE "${Simbody_LIB_DIR}" "\${OpenSim_LIB_DIR}"
            _imploc_new ${_imploc_old})
        set(${varname} "${${varname}}\n    set_target_properties(${targetname} PROPERTIES IMPORTED_LOCATION_${_cfg} \"${_imploc_new}\")")

        foreach(_propname IMPORTED_LINK_INTERFACE_LIBRARIES
                          IMPORTED_SONAME)
            get_property(_prop_is_set TARGET ${targetname} PROPERTY
                ${_propname}_${_cfg} SET)
            if(${_prop_is_set})
                get_property(_propvalue TARGET ${targetname} PROPERTY
                            ${_propname}_${_cfg})
                set(${varname} "${${varname}}\n    set_target_properties(${targetname} PROPERTIES ${_propname}_${_cfg} \"${_propvalue}\")")
            endif()
        endforeach()
    endforeach()
    set(${varname} "${${varname}}\n")
endmacro()
foreach(simbodylib ${Simbody_LIBRARIES})
    OpenSimWriteImportedTarget(OPENSIM_COPY_SIMBODY_IMPORTED_TARGETS
        ${simbodylib})
endforeach()

# The generated file must not be named OpenSimConfig.cmake, otherwise CMake
# will pick it up and think that the build tree contains an OpenSim
# installation.
configure_package_config_file(
    OpenSimConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfigToInstall.cmake"
    INSTALL_DESTINATION "${OPENSIM_INSTALL_CMAKE_DIR}"
    PATH_VARS # Variables to edit in OpenSimConfig.cmake.in.
        CMAKE_INSTALL_PREFIX
        OPENSIM_INSTALL_CMAKE_DIR
    )

# Version file.
write_basic_config_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfigVersion.cmake"
    VERSION "${OPENSIM_VERSION}"
    COMPATIBILITY SameMajorVersion)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfigToInstall.cmake"
    DESTINATION "${OPENSIM_INSTALL_CMAKE_DIR}"
    RENAME OpenSimConfig.cmake
    )

install(
    FILES
        "SampleCMakeLists.txt"
        "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfigVersion.cmake"
    DESTINATION
        "${OPENSIM_INSTALL_CMAKE_DIR}"
        )

install(EXPORT OpenSimTargets DESTINATION "${OPENSIM_INSTALL_CMAKE_DIR}")
