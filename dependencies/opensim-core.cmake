# This file is included by the CMakeLists.txt in this directory.

# We list (and update) the opensim-core submodule commit here so that AppVeyor
# will invalidate its cached opensim-core installation if we change the commit.
# This commented commit hash is not actually used in the superbuild.
# opensim-core commit: a7bf61b894368a9e1a374796a9693248fc3c04ee

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
                  GIT_TAG    187d22c690359677912f03c0490049f878d36c72
                  CMAKE_ARGS -DBUILD_EXAMPLES:BOOL=OFF 
                             -DBUILD_TESTING:BOOL=OFF)

    AddDependency(NAME       docopt
                  GIT_URL    https://github.com/docopt/docopt.cpp.git
                  GIT_TAG    af03fa044ee1eff20819549b534ea86829a24a54
                  CMAKE_ARGS -DCMAKE_DEBUG_POSTFIX:STRING=_d)

    add_dependencies(opensim-core simbody docopt)
endif()


