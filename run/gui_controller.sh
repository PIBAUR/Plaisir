#!/bin/bash
export PS1='\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\u@\h:\w\$'
source ~/.bashrc
roslaunch gui_controller run.launch
