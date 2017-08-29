#add_executable(test_sliding_mass_minimum_effort
#               test_sliding_mass_minimum_effort.cpp)
#
#target_link_libraries(test_sliding_mass_minimum_effort tropter)
#target_include_directories(test_sliding_mass_minimum_effort
#        # TODO tropter shouldn't know that it's in a larger project.
#        PRIVATE "${CMAKE_SOURCE_DIR}/tropter/external/catch")
#
#
#add_executable(test_eigen_adolc_reverse_mode
#               test_eigen_adolc_reverse_model.cpp)

tropter_add_test(test_derivatives tropter)
tropter_add_test(test_eigen_adolc_reverse_mode tropter)
tropter_add_test(test_generic_optimization tropter)
tropter_add_test(test_sliding_mass_minimum_effort tropter)
tropter_add_test(test_sliding_mass_minimum_time tropter)
tropter_add_test(test_analytic_optimal_control_solutions tropter)
tropter_add_test(test_double_pendulum tropter)
tropter_add_test(test_path_constraints tropter)
tropter_add_test(test_optimal_control_initial_guess tropter)
if(MUSCOLLO_WITH_SNOPT)
    tropter_add_test(test_snopt tropter) # snopt7_cpp
endif()

if(OpenMP_FOUND)
    tropter_add_test(test_adolc_parallelized_openmp tropter)
    target_compile_options(test_adolc_parallelized_openmp PRIVATE
            "${OpenMP_CXX_FLAGS}")
    set_property(TARGET test_adolc_parallelized_openmp APPEND PROPERTY
            LINK_FLAGS "${OpenMP_CXX_FLAGS}")
endif()
