#!/bin/bash

this_ip=$(ifconfig | grep "dr:192.168.") # depending on systems, can be addr: or adr:
this_ip=`echo $this_ip | cut -d' ' -f 2`
this_ip=`echo ${this_ip:4}`
this_ip=`echo ${this_ip/:/}`
export THIS_IP=$this_ip
