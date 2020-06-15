# Find the pre-built Fortran SNOPT library.
# On Windows, this can also be used to find the SNOPT library that contains the
# C++ interface, because it's named snopt7 (not snopt7_cpp as on other
# platforms).
# Use the Intel ifort variant on Mac.

set(SNOPT_DIR $ENV{SNOPT_DIR} CACHE PATH
    "Directory containing pre-built snopt7 Fortran library from ccom.ucsd.edu.")

set(SNOPT_LIB_NAME "snopt7")

find_library(SNOPT_LIBRARY ${SNOPT_LIB_NAME}
        DOC "The Fortran library for the SNOPT optimization algorithm"
        HINTS "${SNOPT_DIR}")

add_library(snopt7 STATIC IMPORTED)
set_property(TARGET snopt7 PROPERTY 
    IMPORTED_LOCATION "${SNOPT_LIBRARY}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SNOPT DEFAULT_MSG
                                  SNOPT_LIBRARY)

mark_as_advanced(SNOPT_LIBRARY)

