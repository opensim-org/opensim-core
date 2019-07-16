FROM ubuntu

MAINTAINER Christopher Dembia

# opensim-moco is a private repository on GitHub, so we need permission to
# access the repository within the Docker container. Create a Personal Access
# Token on the GitHub website:
# 1. Click user icon in the upper right hand corner.
# 2. Click Settings.
# 3. Click Developer settings.
# 4. Click Personal access tokens.
# 5. Click Generate new token.
# 6. Give a name to your token, e.g., "opensim-moco Docker"
# 7. Check the box next to "repo"
# 8. Click Generate token.
# 9. Copy the token to the clipboard.
# 10. Run Docker build as follows:
#
#       docker build --build-arg GITHUBTOKEN=<paste> .
#

# TODO: Remove when opensim-moco is public.
ARG GITHUBTOKEN

ARG MOCOBRANCH=master

RUN apt-get update && apt-get install -y \
        git \
        build-essential libtool autoconf \
        cmake \
        gfortran \
        wget \
        pkg-config \
        libopenblas-dev \
        liblapack-dev

# TODO: this embeds the token in git settings!
RUN git config --global url."https://$GITHUBTOKEN:@github.com/".insteadOf "https://github.com/" \
        && git clone https://github.com/stanfordnmbl/opensim-moco.git /opensim-moco \
        && cd /opensim-moco \
        && git checkout $MOCOBRANCH \
        && rm ~/.gitconfig

RUN cd /opensim-moco \
        && git submodule update --init \
        && mkdir ../moco_dependencies_build \
        && cd ../moco_dependencies_build \
        && cmake ../opensim-moco/dependencies \
        && make --jobs 4 ipopt \
        && make --jobs 4 \
        && echo "/moco_dependencies_install/adol-c/lib64" >> /etc/ld.so.conf.d/moco.conf \
        && echo "/moco_dependencies_install/ipopt/lib" >> /etc/ld.so.conf.d/moco.conf \
        && ldconfig \
        && rm -r /moco_dependencies_build

RUN cd / \
        && mkdir build \
        && cd build \
        && cmake ../opensim-moco \
        && make --jobs 4 install \
        && echo "/opensim-moco-install/sdk/lib" >> /etc/ld.so.conf.d/moco.conf \
        && ldconfig \
        && rm -r /build

# TODO build python bindings.

