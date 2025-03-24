#! /bin/bash
script_path=$(cd $(dirname ${0}) && pwd)

IMAGE_NAME="map_cleaner"
CONTAINER_NAME="map_cleaner_container"
ROCKER_OPTION="--x11 --user --privileged"

docker build $script_path -t "$IMAGE_NAME"
rocker $ROCKER_OPTION --volume $script_path/data:/data --name $CONTAINER_NAME -- $IMAGE_NAME