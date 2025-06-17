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
    if [[ "$(uname)" == "Darwin"  ]];then
        # https://trac.ffmpeg.org/wiki/CompilationGuide/macOS
        if ! hash brew 2>/dev/null;then
            echo 'brew not found, please install brew first.'
            echo 'Install example: >> /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"'
            exit
        fi
        brew install automake fdk-aac git lame libass libtool libvorbis libvpx opus sdl shtool texi2html theora wget x264 x265 xvid nasm
    elif [[ "$(uname)" == "Linux" ]]; then
        # Ubuntu/Debian
        sudo apt-get update
        sudo apt-get install -y \
            autoconf \
            automake \
            build-essential \
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
    elif [[ "$(uname)" == *"_NT"* ]]; then
        echo "Not supported windows now."
    fi
}

build_install_ffmpeg() {
    cd $opt
    if [ ! -d ffmpeg ];then
        git clone https://git.ffmpeg.org/ffmpeg.git
    fi
    cd ffmpeg
    git reset --hard b6f84cd7
    git apply $cur/ffmpeg_b6f84cd7.patch
    
    # Clean previous build
    make distclean || true
    
    # Configure with enhanced PIC support
    ./configure \
        --disable-doc \
        --prefix=$INSTALL_PREFIX \
        --pkg-config-flags="--static" \
        --enable-pic \
        --enable-static \
        --extra-cflags="-I/usr/local/include -fPIC -fPIE" \
        --extra-cxxflags="-fPIC -fPIE" \
        --extra-ldflags="-L/usr/local/lib -fPIC" \
        --enable-libass \
        --enable-libfreetype \
        --enable-libvorbis \
        --enable-version3

        # --enable-shared \
        # --enable-libmp3lame \
    
    make -j$(nproc)
    make install
}

install_deps
build_install_ffmpeg
