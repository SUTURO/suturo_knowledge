#!/bin/bash

# MongoDB starten
mongod --config /etc/mongod.conf --fork

sleep  5

source /catkin_ws/devel/setup.bash

roslaunch suturo_knowledge knowrob.launch
