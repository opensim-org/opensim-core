find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

tropter_add_test(NAME test_derivatives LIB_DEPENDS Catch2::Catch2WithMain)
# This test was more of a sandbox for learning ADOL-C:
#tropter_add_test(NAME test_eigen_adolc_reverse_mode)
tropter_add_test(NAME test_generic_optimization LIB_DEPENDS Catch2::Catch2WithMain)

tropter_add_test(NAME test_sliding_mass_minimum_effort LIB_DEPENDS Catch2::Catch2WithMain)
tropter_add_test(NAME test_sliding_mass_minimum_time LIB_DEPENDS Catch2::Catch2WithMain)
tropter_add_test(NAME test_analytic_optimal_control_solutions LIB_DEPENDS Catch2::Catch2WithMain)
tropter_add_test(NAME test_double_pendulum LIB_DEPENDS Catch2::Catch2WithMain)
tropter_add_test(NAME test_path_constraints LIB_DEPENDS Catch2::Catch2WithMain)
tropter_add_test(NAME test_optimal_control_initial_guess LIB_DEPENDS Catch2::Catch2WithMain)
tropter_add_test(NAME test_parameter_optimization LIB_DEPENDS Catch2::Catch2WithMain)
if(TROPTER_WITH_SNOPT)
    tropter_add_test(NAME test_snopt LIB_DEPENDS snopt7_cpp)
endif()

#if(OpenMP_FOUND)
#    tropter_add_test(NAME test_adolc_parallelized_openmp)
#    target_compile_options(test_adolc_parallelized_openmp PRIVATE
#            "${OpenMP_CXX_FLAGS}")
#    set_property(TARGET test_adolc_parallelized_openmp APPEND PROPERTY
#            LINK_FLAGS "${OpenMP_CXX_FLAGS}")
#endif()
