#! /bin/sh
sudo chown -R $(id -u):$(id -g) /MapCleaner
. /MapCleaner/devel/setup.sh
roslaunch map_cleaner run.launch config:=/data/config/config.yaml
