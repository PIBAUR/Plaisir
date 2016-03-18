#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

robots = [robot for robot in sys.argv[1:] if robot != "-p"]

config = """Panels:
  - Class: rviz/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:"""

for robot in robots:
    config += "\n        - /" + robot + "1"
    
config += """
      Splitter Ratio: 0.652866
    Tree Height: 775
  - Class: rviz/Selection
    Name: Selection
  - Class: rviz/Tool Properties
    Expanded:
      - /2D Pose Estimate1
      - /2D Nav Goal1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.588679
  - Class: rviz/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: ""
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz/PointStamped
      Color: 204; 41; 204
      Enabled: true
      History Length: 1
      Name: ClickedPoint
      Radius: 0.2
      Topic: /clicked_point
      Value: true
    - Alpha: 0.7
      Class: rviz/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic: /map
      Value: true"""

for robot in robots:
    config += """
    - Class: rviz/Group
      Displays:
        - Angle Tolerance: 0.05
          Class: rviz/Odometry
          Color: 0; 170; 0
          Enabled: true
          Keep: 300
          Length: 0.1
          Name: """ + robot + """/Odom
          Position Tolerance: 0.05
          Topic: /robot""" + robot + """/odom
          Value: true
        - Arrow Length: 0.3
          Class: rviz/PoseArray
          Color: 0; 255; 255
          Enabled: true
          Name: """ + robot + """/ParticulesAMCL
          Topic: /robot""" + robot + """/particlecloud
          Value: true
        - Arrow Length: 0.2
          Class: rviz/PoseArray
          Color: 170; 0; 255
          Enabled: true
          Name: """ + robot + """/Path
          Topic: /robot""" + robot + """/path_viz
          Value: true
        - Alpha: 1
          Autocompute Intensity Bounds: true
          Autocompute Value Bounds:
            Max Value: 10
            Min Value: -10
            Value: true
          Axis: Z
          Channel Name: intensity
          Class: rviz/LaserScan
          Color: 0; 85; 255
          Color Transformer: FlatColor
          Decay Time: 0
          Enabled: true
          Invert Rainbow: false
          Max Color: 255; 255; 255
          Max Intensity: 46
          Min Color: 0; 0; 0
          Min Intensity: 9
          Name: """ + robot + """/LaserScan
          Position Transformer: XYZ
          Queue Size: 10
          Selectable: true
          Size (Pixels): 3
          Size (m): 0.05
          Style: Spheres
          Topic: /robot""" + robot + """/scan
          Use Fixed Frame: true
          Use rainbow: true
          Value: true
        - Class: rviz/TF
          Enabled: true
          Frame Timeout: 15
          Frames:
            All Enabled: true
            map:
              Value: true
            """ + robot + """/base_laser_link:
              Value: true
            """ + robot + """/base_link:
              Value: true
            """ + robot + """/odom:
              Value: true
          Marker Scale: 1
          Name: """ + robot + """/TF
          Show Arrows: true
          Show Axes: true
          Show Names: true
          Tree:
            map:
              """ + robot + """/odom:
                """ + robot + """/base_link:
                  """ + robot + """/base_laser_link:
                    {}
          Update Interval: 0
          Value: true
      Enabled: false
      Name: """ + robot + """"""
      
config += """
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz/Interact
      Hide Inactive Objects: true
    - Class: rviz/MoveCamera
    - Class: rviz/Select
    - Class: rviz/FocusCamera
    - Class: rviz/Measure
    - Class: rviz/SetInitialPose
      Topic: /initialpose
    - Class: rviz/SetGoal
      Topic: /move_base_simple/goal
    - Class: rviz/PublishPoint
      Single click: true
      Topic: /clicked_point
  Value: true
  Views:
    Current:
      Angle: -0.02
      Class: rviz/TopDownOrtho
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Name: Current View
      Near Clip Distance: 0.01
      Scale: 63.3977
      Target Frame: <Fixed Frame>
      Value: TopDownOrtho (rviz)
      X: 3.17115
      Y: -0.650256
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 1056
  Hide Left Dock: false
  Hide Right Dock: true
  QMainWindow State: 000000ff00000000fd0000000400000000000001ca00000396fc0200000007fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000006400fffffffb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000002800000396000000dd00fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f00000396fc0200000004fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730000000028000000930000006400fffffffb0000000a0056006900650077007300000000c1000002fd000000b000fffffffb0000001e0054006f006f006c002000500072006f0070006500720074006900650073010000029e000001200000000000000000fb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000007800000003efc0100000002fb0000000800540069006d0065010000000000000780000002f600fffffffb0000000800540069006d00650100000000000004500000000000000000000005b00000039600000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Time:
    collapsed: false
  Tool Properties:
    collapsed: true
  Views:
    collapsed: true
  Width: 1920
  X: 1913
  Y: 24
"""

if __name__ == "__main__":
    configFile = open(os.path.expanduser("~") + "/catkin_ws/params/init_global.rviz", "w")
    configFile.write(config)
    configFile.close()