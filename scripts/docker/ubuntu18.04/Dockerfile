FROM ubuntu:bionic
SHELL ["/bin/bash", "-i", "-c"]

# Upgrade repositories.
RUN apt-get update

# Set locales
RUN apt-get install -y locales
RUN locale-gen en_US.UTF-8
ENV LC_ALL en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8

# Install packages.
RUN apt-get install -y git wget build-essential autotools-dev automake bison byacc
RUN apt-get install -y libpcre3 libpcre3-dev
RUN ln -s /usr/lib/x86_64-linux-gnu/libpcre.so /usr/lib/x86_64-linux-gnu/libpcre.so.1

# Exit when an error happens instead of continue.
RUN set -e

# Default values for flags.
ENV DEBUG_TYPE="Release"
ENV NUM_JOBS=8
ENV MOCO="on"
ENV CORE_BRANCH="main"
ENV GENERATOR="Unix Makefiles"

# Show values of flags:
RUN echo
RUN echo "Build script parameters:"
RUN echo "DEBUG_TYPE="$DEBUG_TYPE
RUN echo "NUM_JOBS="$NUM_JOBS
RUN echo "MOCO="$MOCO
RUN echo "CORE_BRANCH="$CORE_BRANCH
RUN echo "GENERATOR="$GENERATOR
RUN echo ""

# Install dependencies from package manager.
RUN echo "LOG: INSTALLING DEPENDENCIES..."
RUN apt-get update && apt-get install --yes build-essential cmake autotools-dev autoconf pkg-config automake libopenblas-dev liblapack-dev freeglut3-dev libxi-dev libxmu-dev doxygen python3 python3-dev python3-numpy python3-setuptools git byacc libssl-dev libpcre3 libpcre3-dev libtool gfortran ninja-build patchelf openjdk-8-jdk p7zip-full wget || ( echo "Installation of dependencies using apt-get failed." && exit )
RUN echo 

# Create workspace folder.
RUN mkdir ~/opensim-workspace || true

RUN cmake --version

# Install cmake >= 3.15, build it from source.
RUN echo "LOG: INSTALLING CMAKE >=3.15..."
RUN mkdir ~/opensim-workspace/cmake-3.23.3-source || true
WORKDIR ~/opensim-workspace/cmake-3.23.3-source
RUN wget -nc -q --show-progress https://github.com/Kitware/CMake/releases/download/v3.23.3/cmake-3.23.3.tar.gz && \
    tar -zxvf cmake-3.23.3.tar.gz && \
    cd cmake-3.23.3 && \
    ./bootstrap && \
    make -j$NUM_JOBS && \
    make install
RUN source ~/.bashrc && cmake --version
RUN echo

# Download and install SWIG 4.0.2.
RUN mkdir -p ~/opensim-workspace/swig-source || true
WORKDIR ~/opensim-workspace/swig-source
RUN wget -nc -q --show-progress https://github.com/swig/swig/archive/refs/tags/rel-4.0.2.tar.gz
RUN tar xzf rel-4.0.2.tar.gz
WORKDIR ./swig-rel-4.0.2
RUN ./autogen.sh && ./configure --prefix=$HOME/swig --disable-ccache
RUN make && make -j$NUM_JOBS install  

# Get opensim-core.
RUN echo "LOG: CLONING OPENSIM-CORE..."
RUN git -C ~/opensim-workspace/opensim-core-source pull || git clone https://github.com/opensim-org/opensim-core.git ~/opensim-workspace/opensim-core-source
RUN cd ~/opensim-workspace/opensim-core-source && \
	git checkout $CORE_BRANCH
RUN echo

# Build opensim-core dependencies.
RUN echo "LOG: BUILDING OPENSIM-CORE DEPENDENCIES..."
RUN mkdir -p ~/opensim-workspace/opensim-core-dependencies-build || true
WORKDIR ~/opensim-workspace/opensim-core-dependencies-build
RUN cd ~/opensim-workspace/opensim-core-dependencies-build && \
	cmake ~/opensim-workspace/opensim-core-source/dependencies -DCMAKE_INSTALL_PREFIX=~/opensim-workspace/opensim-core-dependencies-install/ -DSUPERBUILD_ezc3d=on -DOPENSIM_WITH_CASADI=$MOCO -DOPENSIM_WITH_TROPTER=$MOCO && \
	cmake . -LAH && \
	cmake --build . --config $DEBUG_TYPE -j$NUM_JOBS
RUN echo

# Build opensim-core.
RUN echo "LOG: BUILDING OPENSIM-CORE..."
RUN mkdir -p ~/opensim-workspace/opensim-core-build || true
WORKDIR ~/opensim-workspace/opensim-core-build
RUN cd ~/opensim-workspace/opensim-core-build && \
	cmake ~/opensim-workspace/opensim-core-source -G"$GENERATOR" -DOPENSIM_DEPENDENCIES_DIR=~/opensim-workspace/opensim-core-dependencies-install/ -DBUILD_JAVA_WRAPPING=on -DBUILD_PYTHON_WRAPPING=on -DOPENSIM_C3D_PARSER=ezc3d -DBUILD_TESTING=off -DCMAKE_INSTALL_PREFIX=~/opensim-core -DOPENSIM_INSTALL_UNIX_FHS=off -DSWIG_DIR=~/swig/share/swig -DSWIG_EXECUTABLE=~/swig/bin/swig && \
	cmake . -LAH && \
	cmake --build . --config $DEBUG_TYPE -j$NUM_JOBS
RUN echo

# Test opensim-core.
RUN ls ~/opensim-workspace/opensim-core-dependencies-install/simbody/lib
RUN echo "LOG: TESTING OPENSIM-CORE..."
ENV LD_LIBRARY_PATH=/root/opensim-workspace/opensim-core-dependencies-install/simbody/lib/
WORKDIR ~/opensim-workspace/opensim-core-build
RUN cd ~/opensim-workspace/opensim-core-build && \
	ctest --parallel $NUM_JOBS --output-on-failure

# Install opensim-core.
RUN echo "LOG: INSTALL OPENSIM-CORE..."
WORKDIR ~/opensim-workspace/opensim-core-build
RUN cd ~/opensim-workspace/opensim-core-build && \
	cmake --install .

# Create 7z file containing opensim-core.
RUN mkdir -p ~/artifacts/ || true
WORKDIR ~/artifacts/
RUN cd ~/artifacts/ && \
	7z a ~/artifacts/opensim-core.7z ~/opensim-core/