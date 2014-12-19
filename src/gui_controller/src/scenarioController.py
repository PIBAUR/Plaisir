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
        scale = .6
        aPoint = Point(0 * scale, 0 * scale, 0 * scale)
        bPoint = Point(1 * scale, 0 * scale, 0 * scale)
        cPoint = Point(2 * scale, 1 * scale, 0 * scale)
        dPoint = Point(1 * scale, 2 * scale, 0 * scale)
        ePoint = Point(0 * scale, 1 * scale, 0 * scale)
        fPoint = Point(0 * scale, 0 * scale, 0 * scale)
        self.scenario.header = header
        self.scenario.bezier_paths = BezierPath()
        self.scenario.bezier_paths.curves = [BezierCurve(aPoint, bPoint, Point(aPoint.x + .25 * scale, aPoint.y, 0 * scale), Point(bPoint.x -.25 * scale, bPoint.y -.25 * scale, 0)),
                                             BezierCurve(bPoint, cPoint, Point(bPoint.x + .25 * scale, bPoint.y + .25 * scale, 0), Point(cPoint.x + 0 * scale, cPoint.y -.25 * scale, 0)),
                                             BezierCurve(cPoint, dPoint, Point(cPoint.x + 0 * scale, cPoint.y + .25 * scale, 0), Point(dPoint.x + .5 * scale, dPoint.y + 0 * scale, 0)),
                                             BezierCurve(dPoint, ePoint, Point(dPoint.x -.5 * scale, dPoint.y, 0), Point(ePoint.x + .25 * scale, ePoint.y + .25 * scale, 0)),
                                             BezierCurve(ePoint, fPoint, Point(ePoint.x -.25 * scale, ePoint.y -.25 * scale, 0), Point(fPoint.x -.25 * scale, fPoint.y + .25 * scale, 0)),
                                             ]
        rospy.loginfo(str(self.scenario))
        self.publisher.publish(self.scenario)