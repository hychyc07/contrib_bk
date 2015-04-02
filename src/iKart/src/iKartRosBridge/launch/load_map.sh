#!/usr/bin/env bash

filename=$1

if [ -z $filename ]
then #test if the parameter exists
    filename="default_map"
fi

if [ -e "$filename.yaml" ]
then #test if the file already exists
    rosrun map_server map_server $filename.yaml
else
    echo "Can't start map_server, $filename.yaml not found"
fi

