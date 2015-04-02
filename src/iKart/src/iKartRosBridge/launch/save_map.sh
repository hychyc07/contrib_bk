#!/usr/bin/env bash

filename=$1

if [ -z $filename ]
then #test if the parameter exists
    filename="default_map"
fi

if [ -e "$filename.yaml" ]
then #test if the file already exists
    echo "$filename already exists. overrwite (Y/N)"
    read answer
    if test "$answer" != "Y" -a "$answer" != "y"
       then exit 0;
    fi
fi

#the filename does not exist or overrwrite is requested
rosrun map_server map_saver -f $filename


