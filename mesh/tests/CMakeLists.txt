#add_executable(test_sliding_mass_minimum_effort
#               test_sliding_mass_minimum_effort.cpp)
#
#target_link_libraries(test_sliding_mass_minimum_effort mesh)
#target_include_directories(test_sliding_mass_minimum_effort
#        # TODO MESH shouldn't know that it's in a larger project.
#        PRIVATE "${CMAKE_SOURCE_DIR}/mesh/external/catch")
#
#
#add_executable(test_eigen_adolc_reverse_mode
#               test_eigen_adolc_reverse_model.cpp)

mesh_add_test(test_sliding_mass_minimum_effort)
mesh_add_test(test_eigen_adolc_reverse_mode)
mesh_add_test(test_generic_optimization)
