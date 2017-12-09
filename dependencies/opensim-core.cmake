# This file is included by the CMakeLists.txt in this directory.

AddDependency(NAME       opensim-core
              # StatesTrajectory::createFromStatesStorage() assembles the states.
              URL        https://github.com/opensim-org/opensim-core/archive/e05338dde404e055f300f7685397dc61cc495961.zip
              CMAKE_ARGS -DBUILD_API_EXAMPLES:BOOL=OFF
                         -DBUILD_TESTING:BOOL=OFF
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

