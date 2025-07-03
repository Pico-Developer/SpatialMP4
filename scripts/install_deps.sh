#!/usr/bin/env bash

_curfile=$(realpath $0)
cur=$(dirname $_curfile)

if [[ "$(uname)" == "Darwin"  ]];then
    brew install --formula $cur/cmake.rb
    brew install opencv
elif [[ "$(uname)" == "Linux" ]]; then
    SUDO="sudo"
    if [ $(id -u) -eq 0 ];then
        SUDO=""
    fi
    APT="apt"
    if hash yum 2>/dev/null;then
        APT="yum"
    fi
    ${SUDO} ${APT} update && ${SUDO} ${APT} install -y libopencv-dev cmake
elif [[ "$(uname)" == *"_NT"* ]]; then
    echo "Not supported windows now."
fi
