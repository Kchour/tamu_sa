#!/usr/bin/env bash
# need to include thresholds nexxt time
rosrun map_server map_saver --occ 90 --free 55 map:=/warty/global_planner/costmap/costmap -f warty_experiment_test
