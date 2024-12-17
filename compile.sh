#!/bin/bash

set +e

if [[ "$1" == "clean" ]]; then
    rm -rf build
fi

if [ ! -d "build" ]; then
    mkdir build
fi

cd build
cmake ..
bear -- make -j
cp compile_commands.json ../
cd ..
