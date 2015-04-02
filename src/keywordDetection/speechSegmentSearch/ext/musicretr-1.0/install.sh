#!/bin/bash

rm -rf build
mkdir build
cd build
cmake ..
make

cd create_lib
if [ $# -ne 1 ]
then
    	cmake .
else
	cmake . -DCMAKE_INSTALL_PREFIX=$1
fi
make install
