#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from scenario_msgs.msg import Scenario, BezierPath, VideoPlayer


videoplayer_pub = rospy.Publisher('videoplayer',VideoPlayer)
bpath_pub = rospy.Publisher('bezier_path',BezierPath)

def scenarioCB(data) :
    bcurves_msg = data.bezier_paths
    videoplayer_msg = data.video_player    
    bpath_pub.publish(bcurves_msg)
    videoplayer_publish(videoplayer_msg)



if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('scenario', Scenario, scenarioCB)
    rospy.spin()
