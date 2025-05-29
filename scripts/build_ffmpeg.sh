#!/usr/bin/env bash

set -e

_curfile=$(realpath $0)
cur=$(dirname $_curfile)
opt=$cur/build_ffmpeg
INSTALL_PREFIX=$opt/ffmpeg_install

if [ ! -d $opt ];then
    mkdir -p $opt
fi

install_deps() {
    # Ubuntu/Debian
    sudo apt-get update
    sudo apt-get install -y \
        autoconf \
        automake \
        build-essential \
        cmake \
        git-core \
        libass-dev \
        libfreetype6-dev \
        libgnutls28-dev \
        libmp3lame-dev \
        libsdl2-dev \
        libtool \
        libva-dev \
        libvdpau-dev \
        libvorbis-dev \
        libxcb1-dev \
        libxcb-shm0-dev \
        libxcb-xfixes0-dev \
        pkg-config \
        texinfo \
        wget \
        yasm \
        zlib1g-dev \
        nasm
}

build_install_x265() {
    cd $opt
    git clone https://bitbucket.org/multicoreware/x265_git
    cd x265_git/build/linux
    cmake -G "Unix Makefiles" -DENABLE_SHARED=OFF -DENABLE_HDR10_PLUS=ON -DENABLE_MULTIVIEW=ON ../../source
    make -j$(nproc)
    sudo make install
}

build_install_ffmpeg() {
    cd $opt
    if [ ! -d ffmpeg ];then
        git clone https://git.ffmpeg.org/ffmpeg.git
    fi
    cd ffmpeg
    git reset --hard b6f84cd7
    git apply $cur/ffmpeg_b6f84cd7.patch
    ./configure \
        --prefix=$INSTALL_PREFIX \
        --pkg-config-flags="--static" \
        --extra-cflags="-I/usr/local/include" \
        --extra-ldflags="-L/usr/local/lib" \
        --enable-gpl \
        --enable-libass \
        --enable-libfreetype \
        --enable-libmp3lame \
        --enable-libvorbis \
        --enable-libx265 \
        --enable-version3 \
        --enable-nonfree
    
    make -j$(nproc)
    make install
}

if [[ "$(uname)" != "Linux" ]]; then
    echo "Only support Linux"
    exit
fi

install_deps
build_install_x265
build_install_ffmpeg
