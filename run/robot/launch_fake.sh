#!/bin/bash

robot=$1

user=$USER

#launch robot
roslaunch robot init_fake.launch robot:=$robot
