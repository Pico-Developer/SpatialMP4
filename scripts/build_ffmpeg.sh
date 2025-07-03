#!/usr/bin/env bash

set -e

_curfile=$(realpath $0)
cur=$(dirname $_curfile)
opt=$cur/build_ffmpeg
INSTALL_PREFIX=$opt/ffmpeg_install

PIXI_ROOT=$cur/../.pixi/envs/default
CMAKE_EXTRA_CONFIG=""
if [ -d $PIXI_ROOT ];then
    INSTALL_PREFIX=$PIXI_ROOT
    CMAKE_EXTRA_CONFIG="-DCMAKE_INCLUDE_PATH=${PIXI_ROOT}/include"
fi
echo "INSTALL_PREFIX: $INSTALL_PREFIX"

if [ ! -d $opt ];then
    mkdir -p $opt
fi

if [[ "$(uname)" == "Darwin"  ]];then
    make_args="-j$(sysctl -n hw.ncpu)"
elif [[ "$(uname)" == "Linux" ]]; then
    make_args="-j$(nproc)"
else
    echo "Not supported windows."
    exit 1
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

    # EXTRA_CONFIG="--enable-shared --disable-static"
    EXTRA_CONFIG="--enable-static --disable-shared --enable-pic --disable-x86asm --disable-asm"

    # if [[ "$(uname)" == "Darwin" ]];then
    #     EXTRA_CONFIG="--enable-shared --disable-static"
    # else
    #     EXTRA_CONFIG="--enable-static --disable-shared --pkg-config-flags='--static'"
    # fi
    
    # Configure with enhanced PIC support

    export CFLAGS="-fPIC"
    export CXXFLAGS="-fPIC"
    export ASFLAGS="-DPIC"
    ./configure \
        --prefix=$INSTALL_PREFIX \
        --extra-cflags="-I/usr/local/include -fPIC" \
        --extra-cxxflags="-fPIC" \
        --extra-ldflags="-L/usr/local/lib -fPIC" \
        --enable-libass \
        --enable-libfreetype \
        --enable-libvorbis \
        --enable-version3 \
        --disable-ffplay \
        --disable-doc \
        ${EXTRA_CONFIG}

        # --enable-libmp3lame \
    
    make $make_args V=1
    make install
}

build_install_opencv() {
    if [[ "$(uname)" == "Darwin"  ]];then
        # Darwin don't need to install opencv from source
        return
    fi
    opencv_version="4.5.0"
    cd $opt
    if [ ! -d opencv ];then
        git clone https://github.com/opencv/opencv.git -b ${opencv_version}
    fi

    BUILD_FLAGS="
        -D CMAKE_BUILD_TYPE=RELEASE
        -D BUILD_LIST=core,imgproc,imgcodecs
        -D WITH_TBB=ON
        -D WITH_OPENEXR=ON
        -D WITH_EIGEN=ON
        -D WITH_GSTREAMER=OFF
        -D WITH_OPENCL=OFF
        -D WITH_CUDA=OFF
        -D WITH_QT=OFF
        -D OPENCV_GENERATE_PKGCONFIG=ON
        -D BUILD_DOCS=OFF
        -D BUILD_TESTS=OFF
        -D BUILD_PERF_TESTS=OFF
        -D BUILD_EXAMPLES=OFF
        -D BUILD_opencv_world=ON
        -D BUILD_opencv_imgcodecs=ON
        -D BUILD_opencv_apps=OFF                                      
        -D BUILD_opencv_calib3d=OFF
        -D BUILD_opencv_dnn=OFF
        -D BUILD_opencv_features2d=OFF                                 
        -D BUILD_opencv_flann=OFF 
        -D BUILD_opencv_gapi=OFF
        -D BUILD_opencv_ml=OFF
        -D BUILD_opencv_photo=OFF
        -D BUILD_opencv_shape=OFF
        -D BUILD_opencv_videoio=OFF
        -D BUILD_opencv_videostab=OFF
        -D BUILD_opencv_highgui=OFF
        -D BUILD_opencv_superres=OFF
        -D BUILD_opencv_stitching=OFF
        -D BUILD_opencv_java=OFF
        -D BUILD_opencv_java_bindings_generator=OFF
        -D BUILD_opencv_js=OFF
        -D BUILD_opencv_gpu=OFF
        -D BUILD_opencv_gpuarithm=OFF
        -D BUILD_opencv_gpubgsegm=OFF
        -D BUILD_opencv_python2=OFF
        -D BUILD_opencv_python3=ON
        -D OPENCV_PYTHON3_INSTALL_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())")
        -D PYTHON_EXECUTABLE=$(which python3)
        -D BUILD_SHARED_LIBS=OFF 
        -D OPENCV_GENERATE_PKGCONFIG=ON
        ${CMAKE_EXTRA_CONFIG}
    "

    cd $opt/opencv
    if [ -d release ];then
        rm -rf release
    fi
    if [ -d modules/gapi ];then
        rm -rf modules/gapi
    fi
    mkdir release && cd release
    cmake $BUILD_FLAGS -D CMAKE_INSTALL_PREFIX=$INSTALL_PREFIX ..
    make $make_args # V=1
    make install
}

# install_deps
build_install_ffmpeg
# build_install_opencv