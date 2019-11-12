FROM ubuntu

MAINTAINER Christopher Dembia

ARG MOCOBRANCH=master

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
        python3-matplotlib python3-opencv \
        python3-setuptools \
        ttf-mscorefonts-installer \
        swig

# Matplotlib's default backend requires a DISPLAY / Xserver.
RUN mkdir -p ~/.config/matplotlib && \
    echo 'backend : Agg' >> ~/.config/matplotlib/matplotlibrc && \
    echo 'font.sans-serif : Arial, Helvetica, sans-serif' >> ~/.config/matplotlib/matplotlibrc

# Must be careful to not embed the GitHub token in the image.
RUN git clone https://github.com/opensim-org/opensim-moco.git /opensim-moco \
        && cd /opensim-moco \
        && git checkout $MOCOBRANCH \
        && rm ~/.gitconfig

RUN cd /opensim-moco \
        && git submodule update --init \
        && mkdir ../moco_dependencies_build \
        && cd ../moco_dependencies_build \
        && cmake ../opensim-moco/dependencies \
            -DOPENSIM_PYTHON_WRAPPING=on \
        && make --jobs $(nproc) ipopt \
        && make --jobs $(nproc) \
        && echo "/moco_dependencies_install/adol-c/lib64" >> /etc/ld.so.conf.d/moco.conf \
        && echo "/moco_dependencies_install/ipopt/lib" >> /etc/ld.so.conf.d/moco.conf \
        && ldconfig \
        && rm -r /moco_dependencies_build

RUN cd / \
        && mkdir build \
        && cd build \
        && cmake ../opensim-moco \
            -DMOCO_PYTHON_BINDINGS=on \
            -DBUILD_TESTING=off \
            -DBUILD_EXAMPLES=off \
        && make --jobs $(nproc) install \
        && echo "/opensim-moco-install/sdk/lib" >> /etc/ld.so.conf.d/moco.conf \
        && echo "/opensim-moco-install/sdk/Simbody/lib" >> /etc/ld.so.conf.d/moco.conf \
        && ldconfig \
        && cd /opensim-moco-install/sdk/Python && python3 setup.py install \
        && rm -r /build

