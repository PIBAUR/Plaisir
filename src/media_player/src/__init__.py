#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import threading

import rospy

from std_msgs.msg import Float64 as Float64Msg
from scenario_msgs.msg import Scenario as ScenarioMsg

from selenium import webdriver
from selenium.webdriver.common.keys import Keys

class MediaPlayer():
    def __init__(self):
        # create browser
        self.browser = webdriver.Firefox()
        # load html file
        self.browser.get("file://" + os.path.split(os.path.abspath(__file__))[0] + "/media_player.html")
        # wait for loading complete
        self.browser.execute_script("$(document.body).trigger('load');")
        # set firefof on fullscreen
        self.browser.find_element_by_tag_name("body").send_keys(Keys.F11)
            
        self.currentPlayer = "video_0"
        self.videoMedias = []
        self.playingVideo = None
        self.waitingForVideoEndThread = None


    def mediaCB(self, data):
        self.videoMedias = [mediaData for mediaData in data.medias.medias if mediaData.type == "video"]
        
        rospy.loginfo("Received " + str(len(self.videoMedias)) + " medias")
        
        if len(self.videoMedias) <= 0:
            return
        
        # set the first one
        self.sendMediaPathToBrowser(self.currentPlayer, self.videoMedias[0].path)
        
        self.playMedia(0)
        
    
    def playMedia(self, index):
        if self.waitingForVideoEndThread is not None:
            self.waitingForVideoEndThread.terminated = True
        
        self.playingVideo = self.videoMedias[index]
        otherPlayer = "video_1" if self.currentPlayer == "video_0" else "video_0"
        
        self.browser.execute_script("$('#" + self.currentPlayer + "').css('display', 'block');")
        self.browser.execute_script("document.getElementById('" + self.currentPlayer + "').play();")
        
        # stop other except this
        self.browser.execute_script("$('#" + otherPlayer + "').css('display', 'none');")
        # load next one if exists
        if index + 1 < len(self.videoMedias):
            self.sendMediaPathToBrowser(otherPlayer, self.videoMedias[index + 1].path)
            
            # wait for the end of the videoMedia
            self.waitingForVideoEndThread = WaitingForVideoEndThread(self.browser, self.currentPlayer, self.playMedia, index + 1)
            self.waitingForVideoEndThread.start()
            
        self.currentPlayer = otherPlayer

    
    def pathFeedbackCB(self, data):
        #TODO: control playback speed
        pass
    
    
    def sendMediaPathToBrowser(self, player, mediaPath):
        if os.path.exists(mediaPath):
            self.browser.execute_script("document.getElementById('" + player + "').src = '" + mediaPath + "';")
        else:
            rospy.logerr("Media '" + mediaPath + "' does not exist in the robot")
    
    
class WaitingForVideoEndThread(threading.Thread):
    def __init__(self, browser, player, callback, nextIndex):
        super(WaitingForVideoEndThread, self).__init__()
        
        self.terminated = False
        self.browser = browser
        self.player = player
        self.callback = callback
        self.nextIndex = nextIndex
    
    def run(self):
        while True:
            if self.terminated:
                return
            if self.browser.execute_script("return document.getElementById('" + self.player + "').ended;"):
                self.callback(self.nextIndex)
                return
             
                 
if __name__ == '__main__':
    mediaPlayer = MediaPlayer()
    
    # ros node
    rospy.init_node('media_player', log_level = rospy.INFO)
    rospy.Subscriber('/robot01/scenario', ScenarioMsg, mediaPlayer.mediaCB)
    rospy.Subscriber('/robot01/path_feedback', ScenarioMsg, mediaPlayer.mediaCB)
    rospy.spin()
