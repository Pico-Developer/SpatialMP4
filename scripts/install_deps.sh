#!/usr/bin/env bash


if [[ "$(uname)" == "Darwin"  ]];then
    brew install opencv
elif [[ "$(uname)" == "Linux" ]]; then
    sudo apt update && sudo apt install -y libopencv-dev
elif [[ "$(uname)" == *"_NT"* ]]; then
    echo "Not supported windows now."
fi
