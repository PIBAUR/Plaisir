import rospy

from std_msgs.msg import *
from geometry_msgs.msg import *
from scenario_msgs.msg import *

class ScenarioController():
    def __init__(self):
        self.publisher = rospy.Publisher('scenario', Scenario)
        self.scenario = Scenario()
    
    
    def send(self):
        self.scenario.video_player = VideoPlayer()
        self.scenario.video_player.video_paths = ["test_video.mp4"]
        header = Header()
        header.frame_id = "map"
        header.stamp = rospy.Time.now()
        aPoint = Point(0, 0, 0)
        bPoint = Point(1, 0, 0)
        cPoint = Point(2, 1, 0)
        dPoint = Point(1, 2, 0)
        ePoint = Point(0, 1, 0)
        fPoint = Point(0, 0, 0)
        self.scenario.header = header
        self.scenario.bezier_paths = BezierPath()
        self.scenario.bezier_paths.curves = [BezierCurve(aPoint, bPoint, Point(aPoint.x + .25, aPoint.y, 0), Point(bPoint.x -.25, bPoint.y -.25, 0)),
                                             BezierCurve(bPoint, cPoint, Point(bPoint.x + .25, bPoint.y + .25, 0), Point(cPoint.x + 0, cPoint.y -.25, 0)),
                                             BezierCurve(cPoint, dPoint, Point(cPoint.x + 0, cPoint.y + .25, 0), Point(dPoint.x + .5, dPoint.y + 0, 0)),
                                             BezierCurve(dPoint, ePoint, Point(dPoint.x -.5, dPoint.y, 0), Point(ePoint.x + .25, ePoint.y + .25, 0)),
                                             BezierCurve(ePoint, fPoint, Point(ePoint.x -.25, ePoint.y -.25, 0), Point(fPoint.x -.25, fPoint.y + .25, 0)),
                                             ]
        rospy.loginfo(str(self.scenario))
        self.publisher.publish(self.scenario)