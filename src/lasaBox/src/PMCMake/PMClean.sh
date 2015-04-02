#!/bin/bash
echo "***************************************************"
echo "PMClean: Recursive build-cleaning of directories"
echo "***************************************************"


HOME_PATH=`pwd`;

#if [ "`basename $HOME_PATH`" != "RobotToolKit" ]
#then
#    echo "Error: PMClean.sh should be run in the RobotToolKit"
#    echo "       folder and not from this one: `basename $HOME_PATH`"
#    exit
#fi

while true; do
    echo "Warning: This operation will erase builing files"
    echo "         and cannot be reverted!"
    read -p "Are you sure? [y/N] ? " yn
    case $yn in
        [Yy]* ) break;;
        * )     exit;;
    esac
done

echo "***************************************************"


Recurse () 
{ 
    fold="$1"

    cd $fold

    if [ "$2" = "CMAKE" ]; then
        #if [ -f CMakeLists.txt.pm ]; then
        rm -rf  CMakeFiles CMakeCache.txt cmake_install.cmake Makefile 
        #fi
    fi


    if [ "$2" = "TEMP" ]; then
        rm -if *~
    fi

    cd - >/dev/null

    for d in $fold/*
    do
        if [ -d "$d" ]; then
            ( Recurse "$d" "$2")
        fi;
    done
}

DelSymLinks () 
{ 
    fold="$1"

    for d in $fold/*
    do
        if [ -h "$d" ]; then
            rm -rf $d
        fi;
    done
}


echo "PMClean: Cleaning projet"
make clean 2> /dev/null
#echo "PMClean: Removing projet's simlinks"
#DelSymLinks ./bin
#DelSymLinks ./module
#rm -rf src/include/*


echo "PMClean: Removing CMake files"
Recurse "." CMAKE
rm -rf CMakeCache.txt CMakeFiles cmake_install.cmake Makefile ./bin

echo "PMClean: Removing temporary files"
Recurse "." TEMP


#echo "PMClean: Removing source documentation"
#rm -rf ./doc/html


echo "***************************************************"
echo "PMClean: done"
echo "***************************************************"



