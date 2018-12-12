

set(SNOPT_DIR $ENV{SNOPT_DIR} CACHE PATH "Path to SNOPT install directory")


find_path(SNOPT_INCLUDES 
          NAMES snopt.h
          HINTS "${SNOPT_DIR}/include")

find_library(SNOPT_LIBRARY snopt7
        DOC "The Fortran library for the SNOPT optimization algorithm"
        HINTS "${SNOPT_DIR}/bin")

add_library(snopt7 STATIC IMPORTED)
set_property(TARGET snopt7 PROPERTY IMPORTED_LOCATION "${SNOPT_DIR}/bin/snopt7.lib")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SNOPT DEFAULT_MSG
                                  SNOPT_LIBRARY SNOPT_INCLUDES)

mark_as_advanced(SNOPT_LIBRARY SNOPT_INCLUDES)

