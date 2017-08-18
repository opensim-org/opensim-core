# TODO The following is preferable to changing the language for the
# overall tomu project:
# project(snopt-interface C CXX Fortran)

# TODO cannot use Fortran with Xcode.

# TODO Move to a FindSNOPT.cmake module file.
# We want to find the static library.
# TODO what issues does this cause for licensing??
#set(CMAKE_FIND_LIBRARY_SUFFIXES .a)
find_library(SNOPT_LIBRARY snopt7
        PATH_SUFFIXES lib
        DOC "The Fortran library for the SNOPT optimization algorithm")

if (${SNOPT_LIBRARY} STREQUAL "SNOPT_LIBRARY-NOTFOUND")
    message(WARNING "Could not find SNOPT library. Set CMAKE_LIBRARY_PATH to \
                     the directory containing the library.")
else()
    message(STATUS "SNOPT library located at ${SNOPT_LIBRARY}")

    add_library(snopt7_cpp
        include/snopt.h
        include/snopt_cwrap.h src/snopt_cwrap.c
        include/snoptProblem.hpp src/snoptProblem.cpp
        src/snopt_wrapper.f90
        )

    # Users will not need SNOPT headers, so we do not provide them under the
    # INSTALL_INTERFACE.
    target_include_directories(snopt7_cpp
            PRIVATE include
            INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

    target_link_libraries(snopt7_cpp ${SNOPT_LIBRARY})

endif()
