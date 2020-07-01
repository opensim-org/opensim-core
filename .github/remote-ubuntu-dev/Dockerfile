# This Dockerfile can be used with the adjacent docker-compose.yml file to
# use the CLion IDE on any platform (including Windows and Mac) to build/test
# opensim-core on Linux.
# This Dockerfile is experimental and may require tweaking for CLion remote
# Ubuntu development to work properly.
# https://github.com/shuhaoliu/docker-clion-dev/blob/master/Dockerfile
# Password for user 'debugger' is pwd.
# We assume the build context is the .../opensim-core/dependencies directory.

FROM ubuntu

########################################################
# Essential packages for remote debugging and login in
########################################################

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    apt-utils gcc g++ openssh-server cmake build-essential gdb gdbserver rsync \
    vim

# Taken from - https://docs.docker.com/engine/examples/running_ssh_service/#environment-variables

RUN mkdir /var/run/sshd
RUN echo 'root:root' | chpasswd
RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# SSH login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile

# 22 for ssh server. 7777 for gdb server.
EXPOSE 22 7777

RUN useradd -ms /bin/bash debugger
RUN echo 'debugger:pwd' | chpasswd

########################################################
# Add custom packages and development environment here
########################################################

ARG BTYPE=RelWithDebInfo

# Avoid interactive timezone prompt when installing packages.
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
        git \
        build-essential libtool autoconf \
        cmake \
        gfortran \
        wget \
        pkg-config \
        libopenblas-dev \
        liblapack-dev \
        python3 python3-dev python3-numpy python3-matplotlib python3-setuptools \
        swig

COPY CMakeLists.txt *.cmake /dependencies/

RUN mkdir /opensim_dependencies_build \
        && cd /opensim_dependencies_build \
        && cmake /dependencies \
            -DCMAKE_BUILD_TYPE=$BTYPE \
            -DSUPERBUILD_ezc3d=on \
        && make --jobs $(nproc) \
        && echo "/opensim_dependencies_install/adol-c/lib64" >> /etc/ld.so.conf.d/opensim.conf \
        && echo "/opensim_dependencies_install/ipopt/lib" >> /etc/ld.so.conf.d/opensim.conf \
        && ldconfig \
        && rm -r /opensim_dependencies_build

########################################################

CMD ["/usr/sbin/sshd", "-D"]
