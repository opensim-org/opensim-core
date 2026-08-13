configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/opensim_tests_config.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/opensim-core/tests/opensim_tests_config.h"
    @ONLY
)

add_library(opensim_testing INTERFACE)
target_include_directories(opensim_testing INTERFACE
    "${CMAKE_CURRENT_BINARY_DIR}/generated/"
)
