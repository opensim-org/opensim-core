# This file is included by the CMakeLists.txt in this directory.

AddDependency(NAME       opensim-core
              # StatesTrajectory::createFromStatesStorage() assembles the states.
              URL        https://github.com/opensim-org/opensim-core/archive/7513accaa13a9431cf37e2bbc35ee7111f50baa6.zip
              CMAKE_ARGS -DBUILD_API_EXAMPLES:BOOL=OFF
                         -DBUILD_TESTING:BOOL=OFF
                         -DSIMBODY_HOME:PATH=${CMAKE_INSTALL_PREFIX}/simbody
                         -DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}/docopt)

if(SUPERBUILD_opensim-core)

    # OpenSim (and OpenSim's dependencies).
    AddDependency(NAME simbody
                  GIT_URL https://github.com/simbody/simbody.git
                  GIT_TAG fd5c03115038a7398ed5ac04169f801a2aa737f2
                  CMAKE_ARGS -DBUILD_EXAMPLES:BOOL=OFF 
                             -DBUILD_TESTING:BOOL=OFF)

    AddDependency(NAME       docopt
                  GIT_URL    https://github.com/docopt/docopt.cpp.git
                  GIT_TAG    af03fa044ee1eff20819549b534ea86829a24a54
                  CMAKE_ARGS -DCMAKE_DEBUG_POSTFIX:STRING=_d)

    add_dependencies(opensim-core simbody docopt)
endif()
