#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os

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


    def mediaCB(self, data):
        self.videoMedias = [mediaData for mediaData in data.medias.medias if mediaData.type == "video"]
        
        rospy.loginfo("Received " + str(len(self.videoMedias)) + " medias")
        
        if len(self.videoMedias) <= 0:
            return
        
        # set the first one
        self.browser.execute_script("document.getElementById('" + self.currentPlayer + "').src = '" + self.videoMedias[0].path + "';")
        
        i = 0
        for videoMedia in self.videoMedias:
            self.playingVideo = videoMedia
            otherPlayer = "video_1" if self.currentPlayer == "video_0" else "video_0"
            
            self.browser.execute_script("$('#" + self.currentPlayer + "').css('display', 'block');")
            self.browser.execute_script("document.getElementById('" + self.currentPlayer + "').play();")
            
            # stop other except this
            self.browser.execute_script("$('#" + otherPlayer + "').css('display', 'none');")
            # load next one if exists
            if i + 1 < len(self.videoMedias):
                self.browser.execute_script("document.getElementById('" + otherPlayer + "').src = '" + self.videoMedias[i + 1].path + "';")
            
            # wait for the end of the videoMedia
            while True:
                if self.browser.execute_script("return document.getElementById('" + self.currentPlayer + "').ended;"):
                    break
            
            self.currentPlayer = otherPlayer
            
            i += 1
    
    
    def pathFeedbackCB(self, data):
        #TODO: control playback speed
        pass
    
    
if __name__ == '__main__':
    mediaPlayer = MediaPlayer()
    
    # ros node
    rospy.init_node('media_player', log_level = rospy.INFO)
    rospy.Subscriber('/robot01/scenario', ScenarioMsg, mediaPlayer.mediaCB)
    rospy.Subscriber('/robot01/path_feedback', ScenarioMsg, mediaPlayer.mediaCB)
    rospy.spin()
