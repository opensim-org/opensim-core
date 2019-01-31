# TODO The following is preferable to changing the language for the
# overall moco project:
# project(snopt-interface C CXX Fortran)

find_package(SNOPT MODULE)
set_package_properties(SNOPT PROPERTIES
        URL https://ccom.ucsd.edu/~optimizers/
        TYPE REQUIRED
        PURPOSE "Sparse nonlinear optimization")

if (${SNOPT_LIBRARY} STREQUAL "SNOPT_LIBRARY-NOTFOUND")
    message(FATAL_ERROR
        "Could not find SNOPT library. Set SNOPT_DIR to the directory "
        "containing the library.")
endif()

message(STATUS "SNOPT library located at ${SNOPT_LIBRARY}")

tropter_copy_dlls(DEP_NAME snopt7 DEP_INSTALL_DIR "${SNOPT_DIR}"
    DONT_INSTALL_DLLS)

if(MSVC)

    # MSVC cannot compile a single library containing Fortran and C/C++ code,
    # so we use the pre-built SNOPT binaries that contain the C++ interface.
    # We simply need to provide the headers.

    add_library(snopt7_cpp INTERFACE)

    # Users will not need SNOPT headers, so we do not provide them under the
    # INSTALL_INTERFACE.
    target_include_directories(snopt7_cpp
        INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")

    target_link_libraries(snopt7_cpp INTERFACE ${SNOPT_LIBRARY})

else()

    # TODO The following is preferable to changing the language for the
    # overall muscollo project:
    # project(snopt-interface C CXX Fortran)

    add_library(snopt7_cpp SHARED
        include/snopt.h
        include/snopt_cwrap.h src/snopt_cwrap.c
        include/sqopt_cwrap.h src/sqopt_cwrap.c
        include/snoptProblem.hpp src/snoptProblem.cpp
        src/snopt_wrapper.f90
        src/sqopt_wrapper.f90
        )

    set_target_properties(snopt7_cpp PROPERTIES COMPILE_OPTIONS "")

    # Users will not need SNOPT headers, so we do not provide them under the
    # INSTALL_INTERFACE.
    target_include_directories(snopt7_cpp
        PRIVATE include
        INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

    target_link_libraries(snopt7_cpp ${SNOPT_LIBRARY})

    install(TARGETS snopt7_cpp EXPORT tropter-config
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()

