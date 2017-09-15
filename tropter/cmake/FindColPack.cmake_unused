# - Find ColPack
# Find the native ColPack headers and libraries.
#
#  COLPACK_INCLUDE_DIR  - Where to find <ColPack/ColPackHeaders.h>, etc.
#  COLPACK_FOUND        - True if ColPack found.
#  COLPACK_LIBRARY_DIR  - Where to find the library files
#  COLPACK_LIBRARIES    - Which libraries are available
#
# Copied from https://raw.githubusercontent.com/shogun-toolbox/shogun/shogun_6.0.0/cmake/FindColPack.cmake
# and modified.

set(COLPACK_DIR $ENV{COLPACK_DIR} CACHE PATH
		"Path to ColPack install directory.")

# Look for the header files
find_path(COLPACK_INCLUDE_DIR NAMES ColPack/ColPackHeaders.h
          PATHS /usr/include
               /usr/local/include
               /usr/local/include
               /opt/local/include
               "c:\\libs\\ColPack\\include"
          HINTS ${COLPACK_DIR}/include
          PATH_SUFFIXES ColPack)

# Look for the libraries
find_library(COLPACK_LIBRARIES NAMES ColPack
            PATHS
                   /usr/lib/ColPack
                   /usr/local/lib
                   /opt/local/lib
                   "c:\\libs\\ColPack\\lib"
				   /usr/lib64
                   /usr/local/lib64
                   /usr/local/lib64
                   /opt/local/lib64
                   "c:\\libs\\ColPack\\lib64"
            HINTS ${COLPACK_DIR}/lib
            PATH_SUFFIXES ColPack)

get_filename_component(COLPACK_LIBRARY_DIR ${COLPACK_LIBRARIES} PATH CACHE)

# handle REQUIRED and QUIET options
include (FindPackageHandleStandardArgs)
find_package_handle_standard_args(ColPack REQUIRED_VARS COLPACK_LIBRARIES
        COLPACK_LIBRARY_DIR COLPACK_INCLUDE_DIR)

mark_as_advanced (
  COLPACK_LIBRARIES
  COLPACK_LIBRARY_DIR
  COLPACK_INCLUDE_DIR
)



