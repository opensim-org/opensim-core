#/bin/bash
git submodule update --init
mkdir ../opensim_dependencies_build
cd ../opensim_dependencies_build
cmake ../opensim-core/dependencies
make --jobs 4 ipopt
make --jobs 4
cd ..
mkdir build
cd build
cmake ../opensim-core
make --jobs 4
