#!/bin/bash
export PS1='\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\u@\h:\w\$'
source ~/.bashrc

ssh odroid@192.168.150.101 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate 192.168.150.1'

~/catkin_ws/bin/mapping
