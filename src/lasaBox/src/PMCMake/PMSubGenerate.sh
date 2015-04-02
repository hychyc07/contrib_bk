#!/bin/bash
if [[ $# -lt 1 ]] || [[ $# -gt 2 ]]
then
    echo "Usage: `basename $0` SubProjectName [SubProjectBasePath]"
    echo "  Example: `basename $0` MyProject src/modules"
    exit
fi

BASEPATH=.

if [[ $# -eq 2 ]]
then
    BASEPATH=$2
fi

if [[ ! -d $BASEPATH ]]
then 
    echo "Error: $BASEPATH is not a folder"
    exit
fi

if [ ! -d $BASEPATH/$1 ]
then
    echo "Error: $BASEPATH/$1 is not a folder"
    exit
fi

PROG_PATH=`dirname $0`

if [ -f $BASEPATH/$1/CMakeLists.txt.pm ]
then
    echo Setting up sub-project: $BASEPATH/$1
    sed -e "s/MYLIBNAME/$1/g" < $PROG_PATH/Config.cmake.template > ./$BASEPATH/$1/$1Config.cmake
    grep "PM_PROJECT_DEPS"  ./$BASEPATH/$1/CMakeLists.txt.pm | sed -e "s/PM_PROJECT_DEPS/$1_DEPENDENCIES/g"      >> ./$BASEPATH/$1/$1Config.cmake
    grep "PM_EXTERNAL_DEPS" ./$BASEPATH/$1/CMakeLists.txt.pm | sed -e "s/PM_EXTERNAL_DEPS/$1_EXT_DEPENDENCIES/g" >> ./$BASEPATH/$1/$1Config.cmake
    grep "PM_EXTERNAL_INCDIR" ./$BASEPATH/$1/CMakeLists.txt.pm | sed -e "s/PM_EXTERNAL_INCDIR/$1_INCDIR_DEPENDENCIES/g" >> ./$BASEPATH/$1/$1Config.cmake
    grep "PM_EXTERNAL_LIBS" ./$BASEPATH/$1/CMakeLists.txt.pm | sed -e "s/PM_EXTERNAL_LIBS/$1_LIBS_DEPENDENCIES/g" >> ./$BASEPATH/$1/$1Config.cmake
    grep "PM_EXTERNAL_LIBDIR" ./$BASEPATH/$1/CMakeLists.txt.pm | sed -e "s/PM_EXTERNAL_LIBDIR/$1_LIBDIR_DEPENDENCIES/g" >> ./$BASEPATH/$1/$1Config.cmake
    echo "" >> ./$BASEPATH/$1/$1Config.cmake
    sed -n "/#BEGIN USER_DEFINED ADDITIONAL OPTIONS/,/!./{p;/#END USER_DEFINED ADDITIONAL OPTIONS/q}" < ./$BASEPATH/$1/CMakeLists.txt.pm >> ./$BASEPATH/$1/$1Config.cmake

    cat ./$BASEPATH/$1/CMakeLists.txt.pm $PROG_PATH/CMakeLists.txt.template > ./$BASEPATH/$1/CMakeLists.txt
else
    echo "Error: $BASEPATH/$1/CMakeLists.txt.pm doesn't exists"
    exit
fi


