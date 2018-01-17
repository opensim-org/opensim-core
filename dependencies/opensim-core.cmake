# This file is included by the CMakeLists.txt in this directory.

# We list (and update) the opensim-core submodule commit here so that AppVeyor
# will invalidate its cached opensim-core installation if we change the commit.
# This commented commit hash is not actually used in the superbuild.
# opensim-core commit: 766f5a1354269b7f9872aac6502b1eec12ce3398

AddDependency(NAME       opensim-core
              URL        ${CMAKE_SOURCE_DIR}/../opensim-core
              CMAKE_ARGS -DBUILD_API_EXAMPLES:BOOL=OFF
                         -DBUILD_TESTING:BOOL=OFF
                         -DBUILD_JAVA_WRAPPING:BOOL=${OPENSIM_JAVA_WRAPPING}
                         -DBUILD_PYTHON_WRAPPING:BOOL=${OPENSIM_PYTHON_WRAPPING}
                         -DSIMBODY_HOME:PATH=${CMAKE_INSTALL_PREFIX}/simbody
                         -DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}/docopt)

if(SUPERBUILD_opensim-core)

    # OpenSim's dependencies.
    AddDependency(NAME simbody
                  GIT_URL    https://github.com/simbody/simbody.git
                  GIT_TAG    1fb07d6b0725f595a03065cb3343af485018a439
                  CMAKE_ARGS -DBUILD_EXAMPLES:BOOL=OFF 
                             -DBUILD_TESTING:BOOL=OFF)

    AddDependency(NAME       docopt
                  GIT_URL    https://github.com/docopt/docopt.cpp.git
                  GIT_TAG    af03fa044ee1eff20819549b534ea86829a24a54
                  CMAKE_ARGS -DCMAKE_DEBUG_POSTFIX:STRING=_d)

    add_dependencies(opensim-core simbody docopt)
endif()

