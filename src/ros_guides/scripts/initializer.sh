#!/bin/bash

gnome-terminal -- bash -c "roscore"
gnome-terminal -- bash -c "rosrun ros_guides draw_circle.py"
gnome-terminal -- bash -c "rosrun ros_guides my_first_node.py"
