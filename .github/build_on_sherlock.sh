module load cmake
module load llvm
module load openblas
module load gcc/8.1.0
export CC=gcc
export CXX=g++

cd ../
bash build_on_linux.sh
