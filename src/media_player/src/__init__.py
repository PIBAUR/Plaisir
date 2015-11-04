#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import subprocess
from threading import Thread
import signal

import rospy

from simpleWebSocketServer.SimpleWebSocketServer import SimpleWebSocketServer
from websocket import StreamingWebSocketServer

from std_msgs.msg import Float64 as Float64Msg
from scenario_msgs.msg import Scenario as ScenarioMsg
from scenario_msgs.msg import PathFeedback as PathFeedbackMsg


class MediaPlayer():
    def __init__(self):
        self.currentPlayer = "video_0"
        self.videoMedias = []
        self.playingVideo = None
        self.waitingForVideoEndThread = None
        self.mediaPlayerClientInitialized = False
        
        self.browser = None
        
        # load html file
        mediaPlayerUrl = "file://" + os.path.split(os.path.abspath(__file__))[0] + "/media_player.html"
        self.process = subprocess.Popen(['chromium-browser', "--disable-session-crashed-bubble", "--disable-infobars", "--kiosk", mediaPlayerUrl])
        #subprocess.Popen(['chromium-browser', mediaPlayerUrl])
        
        # start communication
        self.webSocketServer = SimpleWebSocketServer("127.0.0.1", 9001, StreamingWebSocketServer, self.handleBrowserMessageReceived)
        Thread(target = self.webSocketServer.serveforever).start()
        
    
    def handleBrowserMessageReceived(self, message, browser = None):
        if message == "media_player_client_initialized":
            self.mediaPlayerClientInitialized = True
            self.browser = browser
            rospy.loginfo("chromium browser opened with pid: " + str(self.process.pid))


    def mediaCB(self, data):
        if self.mediaPlayerClientInitialized:
            self.videoMedias = [mediaData for mediaData in data.medias.medias if mediaData.type == "video"]
            
            mediaPaths = [videoMediaData.path for videoMediaData in self.videoMedias]
            
            rospy.loginfo("Received " + str(len(self.videoMedias)) + " medias: " + "\n".join(mediaPaths))
            
            if len(self.videoMedias) > 0:
                for mediaPath in mediaPaths:
                    if not os.path.exists(mediaPath):
                        rospy.logerr("Media '" + mediaPath + "' does not exist in the robot")
                
                self.browser.sendMedias(mediaPaths)
        
    
    def pathFeedbackCB(self, data):
        #TODO: control playback speed
        pass
    
    
    def destroy(self):
        self.webSocketServer.close()
                
    
if __name__ == '__main__':
    mediaPlayer = MediaPlayer()
    
    # ros node
    rospy.init_node('media_player', log_level = rospy.INFO)
    rospy.Subscriber('scenario', ScenarioMsg, mediaPlayer.mediaCB)
    rospy.Subscriber('path_feedback', PathFeedbackMsg, mediaPlayer.pathFeedbackCB)
    
    duration = rospy.Duration(.1)
    while not rospy.is_shutdown():
        rospy.sleep(duration)
    
    mediaPlayer.destroy()
