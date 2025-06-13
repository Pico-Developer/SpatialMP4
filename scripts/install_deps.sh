#!/usr/bin/env bash

_curfile=$(realpath $0)
cur=$(dirname $_curfile)

if [[ "$(uname)" == "Darwin"  ]];then
    brew install --formula $cur/cmake.rb
    brew install opencv
elif [[ "$(uname)" == "Linux" ]]; then
    sudo apt update && sudo apt install -y libopencv-dev cmake
elif [[ "$(uname)" == *"_NT"* ]]; then
    echo "Not supported windows now."
fi
