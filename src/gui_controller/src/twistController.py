import rospy

from std_msgs.msg import *
from geometry_msgs.msg import *
from scenario_msgs.msg import *

class TwistController():
    def __init__(self):
        self.publisher = rospy.Publisher('cmd_vel', Twist)
        self.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    
    
    def move(self, linear, angular):
        self.twist.linear.x = linear
        self.twist.angular.z = angular
        rospy.loginfo(str(self.twist))
        self.publisher.publish(self.twist)