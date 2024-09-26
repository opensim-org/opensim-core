cmake_minimum_required(VERSION 3.12)
project(EnzymeTest)

add_executable(enzyme_test enzyme_test.cpp)
target_compile_definitions(enzyme_test PUBLIC -DNOVERSION)
target_link_libraries(enzyme_test PUBLIC ClangEnzymeFlags)