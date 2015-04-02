#!/bin/bash

tar -zxvf qhull-2010.1-src.tgz
cd qhull-2010.1
cmake .
make
cd ..
mkdir include
cp qhull-2010.1/src/*.h include
cp qhull-2010.1/src/libqhull.a .

