#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy 
import tf

#os.system("export PS1='\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\u@\h:\w\$'")
#os.system("source ~/.bashrc")

if __name__=='__main__':
    rospy.init_node('tf_watch_dog',anonymous=True)
    rospy.loginfo("Waiting for init 5 sec...")
    rospy.sleep(5)
    
    baseFrame = "/base_link"
    mapFrame = "/map"
    
    try:
        rospy.get_param("tf_prefix")
        tfPrefix=rospy.get_param("tf_prefix")
        baseFrame = "/"+str(tfPrefix)+str(baseFrame)
    except :
        pass
    rospy.loginfo("Start tf_watch_dog with tf : " + str(baseFrame))
    
    tfListener = tf.TransformListener()
    tf_time = None
    
    loop = rospy.Rate(0.5)
    rospy.sleep(1)
    timeout = True
    while not rospy.is_shutdown() and timeout:
        try:
            tf_time = tfListener.getLatestCommonTime(str(mapFrame), str(baseFrame))
            print(str(tf_time))
            timeout = (rospy.Time().now() - tf_time).to_sec() < 5.0
            rospy.loginfo("tf.timestamp : " + str(tf_time.to_sec()) + " now is : " + str(rospy.Time().now().to_sec()) +" so timeout is : " +str(timeout)) 
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Unable to get tf between %s and %s",mapFrame, baseFrame) 
            rospy.sleep(1)
            pass
        loop.sleep()
    rospy.logerr(" TF between " + mapFrame + " and " + baseFrame + " is too old (> 5 secs). "
                 " MD25's node may be down or may have crashed..."
                 " All nodes will restart." )

