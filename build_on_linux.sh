#/bin/bash
git submodule update --init
mkdir ../moco_dependencies_build
cd ../moco_dependencies_build
cmake ../opensim-moco/dependencies
make --jobs 4 ipopt
make --jobs 4
cd ..
mkdir build
cd build
cmake ../opensim-moco 
make --jobs 4
