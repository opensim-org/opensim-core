# This file is included by the CMakeLists.txt in this directory.

# We list (and update) the opensim-core submodule commit here so that AppVeyor
# will invalidate its cached opensim-core installation if we change the commit.
# This commented commit hash is not actually used in the superbuild.
# opensim-core commit:
# b0222c20bba068aa1abedde4b7d3ac7fba221629

AddDependency(NAME       opensim-core
              URL        ${CMAKE_SOURCE_DIR}/../opensim-core
              CMAKE_ARGS
                    -DBUILD_API_EXAMPLES:BOOL=OFF
                    -DBUILD_TESTING:BOOL=OFF
                    -DBUILD_JAVA_WRAPPING:BOOL=${OPENSIM_JAVA_WRAPPING}
                    -DBUILD_PYTHON_WRAPPING:BOOL=${OPENSIM_PYTHON_WRAPPING}
                    -DOPENSIM_PYTHON_VERSION:STRING=${OPENSIM_PYTHON_VERSION}
                    -DOPENSIM_DEPENDENCIES_DIR:PATH=${CMAKE_INSTALL_PREFIX}
                    -DOPENSIM_C3D_PARSER:STRING=BTK
                    -DOPENSIM_INSTALL_UNIX_FHS:BOOL=OFF)


if(SUPERBUILD_opensim-core)

    # OpenSim's dependencies.
    AddDependency(NAME       BTK
                  GIT_URL    https://github.com/opensim-org/BTKCore.git
                  GIT_TAG    6d787d0be223851a8f454f2ee8c7d9e47b84cbbe
                  CMAKE_ARGS -DBUILD_SHARED_LIBS:BOOL=ON)

    AddDependency(NAME simbody
                  GIT_URL    https://github.com/simbody/simbody.git
                  GIT_TAG    Simbody-3.7
                  CMAKE_ARGS -DBUILD_EXAMPLES:BOOL=OFF 
                             -DBUILD_TESTING:BOOL=OFF)

    AddDependency(NAME       docopt
                  GIT_URL    https://github.com/docopt/docopt.cpp.git
                  GIT_TAG    3dd23e3280f213bacefdf5fcb04857bf52e90917
                  CMAKE_ARGS -DCMAKE_DEBUG_POSTFIX:STRING=_d)

    AddDependency(NAME       spdlog
                  DEFAULT    ON
                  GIT_URL    https://github.com/gabime/spdlog.git
                  GIT_TAG    v1.4.1
                  CMAKE_ARGS -DSPDLOG_BUILD_BENCH:BOOL=OFF
                             -DSPDLOG_BUILD_TESTS:BOOL=OFF
                             -DSPDLOG_BUILD_EXAMPLE:BOOL=OFF
                             -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON)

    add_dependencies(opensim-core BTK simbody docopt spdlog)
endif()



