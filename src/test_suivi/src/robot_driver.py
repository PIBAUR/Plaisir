#!/usr/bin/env python

import roslib
import rospy
from math import sin,cos

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

from neato_driver.neato_driver import xv11, BASE_WIDTH, MAX_SPEED

class RobotDriver:

    def __init__(self):
        """ Start up connection to the Neato Robot. """
        rospy.init_node('robot_driver')

        self.port = rospy.get_param('~port', "/dev/ttyACM0")
        rospy.loginfo("Using port: %s"%(self.port))

        self.robot = xv11(self.port)

        rospy.Subscriber("command_sent", Twist, self.CommandCb)
        self.scanPub = rospy.Publisher('base_scan', LaserScan)
        self.odomPub = rospy.Publisher('odom',Odometry)
        self.odomBroadcaster = TransformBroadcaster()

        self.cmd_vel = [0,0] 

    def spin(self):        
        encoders = [0,0]

        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0
        then = rospy.Time.now()

        # things that don't ever change
        scan_link = rospy.get_param('~frame_id','base_laser_link')
        scan = LaserScan(header=rospy.Header(frame_id=scan_link)) 
        scan.angle_min = 0
        scan.angle_max = 6.26
        scan.angle_increment = 0.017437326
        scan.range_min = 0.020
        scan.range_max = 5.0
        odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_link')
        
        # move out from loop to increase speed
        quaternion = Quaternion()
        odom.pose.pose.position.z = 0
        
        # main loop of driver
        r = rospy.Rate(5)
        self.robot.requestScan()
        while not rospy.is_shutdown():
            # prepare laser scan
            scan.header.stamp = rospy.Time.now()    
            #self.robot.requestScan()
            scan.ranges = self.robot.getScanRanges()

            # get motor encoder values
            left, right = self.robot.getMotors()

            # send updated movement commands
            self.robot.setMotors(self.cmd_vel[0], self.cmd_vel[1], max(abs(self.cmd_vel[0]),abs(self.cmd_vel[1])))
            
            # ask for the next scan while we finish processing stuff
            self.robot.requestScan()
            
            # now update position information
            dt = (scan.header.stamp - then).to_sec()
            then = scan.header.stamp
            d_left = (left - encoders[0])/1000.0
            d_right = (right - encoders[1])/1000.0
            encoders = [left, right]
            
            dx = (d_left+d_right)/2
            dth = (d_right-d_left)/(BASE_WIDTH/1000.0)
            print '{}\t\t{} '.format(self.th,dth)
            x = cos(dth)*dx
            y = -sin(dth)*dx
            self.x += cos(self.th)*x - sin(self.th)*y
            self.y += sin(self.th)*x + cos(self.th)*y
            self.th += dth

            # prepare tf from base_link to odom
            
            quaternion.z = sin(self.th/2.0)
            quaternion.w = cos(self.th/2.0)

            # prepare odometry
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = dx/dt
            odom.twist.twist.angular.z = dth/dt

            # publish everything
            self.odomBroadcaster.sendTransform( (self.x, self.y, 0), (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                then, "base_link", "odom")
            self.scanPub.publish(scan)
            self.odomPub.publish(odom)

            # wait, then do it again
            r.sleep()

        # shut down
        self.robot.setLDS("off")
        self.robot.setTestMode("off") 

    def CommandCb(self,req):
        x = req.linear.x * 1000
        th = req.angular.z * (BASE_WIDTH/2) 
        k = max(abs(x-th),abs(x+th))
        # sending commands higher than max speed will fail
        if k > MAX_SPEED:
            x = x*MAX_SPEED/k; th = th*MAX_SPEED/k
        self.cmd_vel = [ int(x-th) , int(x+th) ]



if __name__ == "__main__":
    robotDriver = RobotDriver()
    robotDriver.spin()
