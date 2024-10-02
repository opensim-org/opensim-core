cmake_minimum_required(VERSION 3.16)
project(EnzymeTest)

add_executable(enzyme_test enzyme_test.cpp)
target_compile_definitions(enzyme_test PUBLIC -DNOVERSION)
target_link_libraries(enzyme_test PUBLIC LLDEnzymeFlags)

add_executable(sandboxEnzyme sandboxEnzyme.cpp)
target_compile_definitions(sandboxEnzyme PUBLIC -DNOVERSION)
# TODO: I assume LLDEnzymeFlags adds in -flto
target_link_libraries(sandboxEnzyme PUBLIC LLDEnzymeFlags osimCommon osimActuators)

add_executable(examplePolynomialFunction examplePolynomialFunction.cpp)
target_compile_definitions(examplePolynomialFunction PUBLIC -DNOVERSION)
target_link_libraries(examplePolynomialFunction PUBLIC LLDEnzymeFlags osimCommon)