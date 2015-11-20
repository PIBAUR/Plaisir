robots_base_ip=$(cat ~/catkin_ws/params/myparams.yaml | grep robots_base_ip)
robots_base_ip=`echo $robots_base_ip| cut -d'"' -f 2`
export ROBOTS_BASE_IP=$robots_base_ip