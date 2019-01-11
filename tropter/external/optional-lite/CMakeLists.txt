add_library(optional-lite INTERFACE)
target_sources(optional-lite INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tropter/optional-lite/optional.hpp>)
target_include_directories(optional-lite INTERFACE
        # Used by other targets in this project:
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        # Used by client projects using an installed version of tropter.
        $<INSTALL_INTERFACE:${TROPTER_INSTALL_INCLUDEDIR}>)

install(TARGETS optional-lite EXPORT tropter-config
        INCLUDES DESTINATION ${TROPTER_INSTALL_INCLUDEDIR})
install(DIRECTORY tropter/ DESTINATION
        ${TROPTER_INSTALL_INCLUDEDIR})
