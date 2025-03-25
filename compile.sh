#!/bin/bash

rm -rf build
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cd ..
rm -rf build-debug
mkdir -p build-debug
cd build-debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
