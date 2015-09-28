#!/usr/bin/env python

""" --------------------------------------------------------------------
    TO CHANGE DEPENDING ON THE PACKAGE:
    DON'T FORGET TO CREATE AN launch/eclipse.launch WITH THE CORRECT PACKAGE
"""
NODE_NAME = "power_manager"
""" -------------------------------------------------------------------- """

import rospy
from std_msgs.msg import Float64

RATE = 0.2 # 0.2Hz --> Periode = 5 sec

def execute():
    pub = rospy.Publisher('battery_percent', Float64)
    rospy.init_node('execute', anonymous=True)
    rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        path = '/sys/devices/12d10000.adc/iio:device0/in_voltage0_raw'
        result = open(path, "r").read()
        result = result.replace("\r", "").replace("\n", "")
        result = float(result)/40.95 #conversion in percent
        battery_percent = result % rospy.get_time()
        rospy.loginfo(battery_percent)
        pub.publish(battery_percent)
        rate.sleep()
 
if __name__ == '__main__':
    try:
        execute()
    except rospy.ROSInterruptException:
        pass
