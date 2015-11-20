#!/bin/bash

this_ip=$(ifconfig | grep "adr:192.168.1")
this_ip=`echo $this_ip | cut -d' ' -f 2`
this_ip=`echo ${this_ip:4}`
export THIS_IP=$this_ip