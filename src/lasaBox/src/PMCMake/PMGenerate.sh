#!/bin/bash

QUIET=0

while getopts 'q' OPTION
do
    case $OPTION in
    q)	QUIET=1 ;;
    esac
done


if [ $QUIET -eq 0 ]; then
    echo "***************************************************"
    echo "PMGenerate: Recursing directories and "
    echo "            generating CMakeLists.txt"
    echo "***************************************************"
fi

PROG_PATH=`dirname $0`

recurse () 
{ 
    local GEN_COUNT=0
    fold="$1"
    if [ -f $fold/CMakeLists.txt.pm ]
    then
        if [ $fold/CMakeLists.txt.pm -nt $fold/CMakeLists.txt ] || [ $PROG_PATH/CMakeLists.txt.template -nt $fold/CMakeLists.txt ]
        then
            $PROG_PATH/PMSubGenerate.sh `basename $fold` `dirname $fold`
            GEN_COUNT=$[$GEN_COUNT + 1]
        fi

        if [ -f $fold/manifest.xml ] && [ $fold/manifest.xml -nt $fold/CMakeLists.txt ]
        then
            touch $fold/CMakeLists.txt
            GEN_COUNT=$[$GEN_COUNT + 1]
        fi
    fi

    for d in $fold/*
    do
        if [ -d "$d" ]; then
            (recurse "$d")            
            GEN_COUNT=$[$GEN_COUNT + $?]
        fi;
    done
    return $GEN_COUNT
}
recurse "."
GEN_COUNT=$?

if [ $QUIET -eq 0 ]; then
    echo "PMGenerate: done"
    echo "***************************************************"
fi
#exit $GEN_COUNT


