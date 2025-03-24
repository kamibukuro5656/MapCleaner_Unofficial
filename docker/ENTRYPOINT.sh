#! /bin/sh
sudo chown -R $(id -u):$(id -g) /MapCleaner
cp /data/config/config.yaml /MapCleaner/src/config/config.yaml
. /MapCleaner/devel/setup.sh
roslaunch map_cleaner run.launch