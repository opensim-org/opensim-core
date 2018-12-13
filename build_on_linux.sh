#/bin/bash
git submodule update --init
mkdir ../moco_dependencies_build
cd ../moco_dependencies_build
cmake ../moco/dependencies
make --jobs 4
mkdir ../moco_build
cd ../moco_build
cmake ../moco 
make --jobs 4
