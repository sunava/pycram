#!/bin/bash

source ${PYCRAM_WS}/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch config_file:=${PYCRAM_WS}/src/pycram/binder/rvizweb_config.json &
roslaunch --wait pycram ik_and_description.launch &
roslaunch --wait pycram giskard_pr2.launch &
cp ${PYCRAM_WS}/src/pycram/binder/webapps.json ${PYCRAM_WS}/src/rvizweb/webapps/app.json 

xvfb-run exec "$@"
