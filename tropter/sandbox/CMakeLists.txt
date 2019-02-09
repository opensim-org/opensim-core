add_executable(sandbox_finite_diff_colpack EXCLUDE_FROM_ALL
        sandbox_finite_diff_colpack.cpp)
set_target_properties(sandbox_finite_diff_colpack PROPERTIES
        FOLDER "tropter/sandbox")

add_executable(sandbox_contact EXCLUDE_FROM_ALL
        sandbox_contact.cpp)
target_link_libraries(sandbox_contact tropter)
set_target_properties(sandbox_contact PROPERTIES
        FOLDER "tropter/sandbox")

add_executable(sandbox_casadi EXCLUDE_FROM_ALL
        sandbox_casadi.cpp)
target_link_libraries(sandbox_casadi tropter casadi)
set_target_properties(sandbox_contact PROPERTIES
        FOLDER "tropter/sandbox")
