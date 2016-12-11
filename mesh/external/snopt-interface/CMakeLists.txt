# TODO The following is preferable to changing the language for the
# overall tomu project:
# project(snopt-interface C CXX Fortran)

# TODO Move to a FindSNOPT.cmake module file.
find_library(SNOPT_LIBRARY snopt7
        PATH_SUFFIXES lib
        DOC "The Fortran library for the SNOPT optimization algorithm")

if (${SNOPT_LIBRARY} STREQUAL "SNOPT_LIBRARY-NOTFOUND")
    message(WARNING "Could not find SNOPT library. Set CMAKE_LIBRARY_PATH to \
                     the directory containing the library.")
else()

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

    # TODO do not hardcode this path.
    target_link_libraries(snopt7_cpp ${SNOPT_LIBRARY})
    #        /Users/chris/nmblprojects/tomu/snopt/snopt7/lib/libsnopt7.dylib
    #        )

endif()
