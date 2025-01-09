FROM ubuntu

MAINTAINER Christopher Dembia

ARG BRANCH=master

RUN echo ttf-mscorefonts-installer msttcorefonts/accepted-mscorefonts-eula \
        select true | debconf-set-selections

# Set DEBIAN_FRONTEND to avoid interactive timezone prompt when installing
# packages.
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        git \
        build-essential libtool autoconf \
        cmake \
        gfortran \
        wget \
        pkg-config \
        libopenblas-dev \
        liblapack-dev \
        python3 python3-dev python3-numpy python3-scipy \
        python3-matplotlib \
        python3-setuptools \
        openjdk-8-jdk \
        ttf-mscorefonts-installer \
        swig3.0

# Matplotlib's default backend requires a DISPLAY / Xserver.
RUN mkdir -p ~/.config/matplotlib && \
    echo 'backend : Agg' >> ~/.config/matplotlib/matplotlibrc && \
    echo 'font.sans-serif : Arial, Helvetica, sans-serif' >> ~/.config/matplotlib/matplotlibrc

RUN git clone https://github.com/opensim-org/opensim-core.git /opensim-core \
        && cd /opensim-core \
        && git checkout $BRANCH

RUN cd /opensim-core \
        && mkdir ../opensim_dependencies_build \
        && cd ../opensim_dependencies_build \
        && cmake ../opensim-core/dependencies \
            -DSUPERBUILD_ezc3d=on \
        && make --jobs $(nproc) \
        && echo "/opensim_dependencies_install/adol-c/lib64" >> /etc/ld.so.conf.d/opensim.conf \
        && echo "/opensim_dependencies_install/ipopt/lib" >> /etc/ld.so.conf.d/opensim.conf \
        && ldconfig \
        && rm -r /opensim_dependencies_build

RUN cd / \
        && mkdir build \
        && cd build \
        && cmake ../opensim-core \
            -DOPENSIM_DEPENDENCIES_DIR=/opensim_dependencies_install \
            -DBUILD_PYTHON_WRAPPING=on \
            -DBUILD_JAVA_WRAPPING=on \
            -DBUILD_TESTING=off \
            -DBUILD_EXAMPLES=off \
            -DOPENSIM_INSTALL_UNIX_FHS=off \
        && make --jobs $(nproc) install \
        && echo "/opensim-core-install/sdk/lib" >> /etc/ld.so.conf.d/opensim.conf \
        && echo "/opensim-core-install/sdk/Simbody/lib" >> /etc/ld.so.conf.d/opensim.conf \
        && ldconfig \
        && cd /opensim-core-install/sdk/Python && python3 setup.py install \
        && rm -r /build

