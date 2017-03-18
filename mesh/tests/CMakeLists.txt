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

mesh_add_test(test_eigen_adolc_reverse_mode mesh)
mesh_add_test(test_generic_optimization mesh)
mesh_add_test(test_sliding_mass_minimum_effort mesh)
mesh_add_test(test_sliding_mass_minimum_time mesh)
mesh_add_test(test_analytic_optimal_control_solutions mesh)
mesh_add_test(test_double_pendulum mesh)
mesh_add_test(test_path_constraints mesh)
mesh_add_test(test_optimal_control_initial_guess mesh)
if(TOMU_WITH_SNOPT)
    mesh_add_test(test_snopt mesh) # snopt7_cpp
endif()

if(OpenMP_FOUND)
    mesh_add_test(test_adolc_parallelized_openmp mesh)
    target_compile_options(test_adolc_parallelized_openmp PRIVATE
            "${OpenMP_CXX_FLAGS}")
    set_property(TARGET test_adolc_parallelized_openmp APPEND PROPERTY
            LINK_FLAGS "${OpenMP_CXX_FLAGS}")
endif()
