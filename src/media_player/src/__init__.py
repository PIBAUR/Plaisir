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
        # get medias
        mediaDirectory = os.path.expanduser("~") + "/videotest/"
        mediaPaths = os.listdir(mediaDirectory)
        videos = []
        for mediaPath in mediaPaths:
            if mediaPath.split(".")[-1] in ["mp4", "mov"]:
                videos.append("file://" + os.path.join(mediaDirectory, mediaPath))
                
        # create browser
        thisDirectoryPath = os.path.abspath("")
        browser = webdriver.Firefox()
        # load html file
        browser.get("file://" + thisDirectoryPath + "/media_player.html")
        # wait for loading complete
        browser.execute_script("$(document.body).trigger('load');")
        # set firefof on fullscreen
        browser.find_element_by_tag_name("body").send_keys(Keys.F11)
        
        currentPlayer = "video_0"
        print "document.getElementById(" + currentPlayer + ").src = '" + videos[0] + "';"
        browser.execute_script("document.getElementById('" + currentPlayer + "').src = '" + videos[0] + "';")
        # execute
        i = 0
        for video in videos:
            otherPlayer = "video_1" if currentPlayer == "video_0" else "video_0"
            
            browser.execute_script("$('#" + currentPlayer + "').css('display', 'block');")
            browser.execute_script("document.getElementById('" + currentPlayer + "').play();")
            
            # stop other except this
            browser.execute_script("$('#" + otherPlayer + "').css('display', 'none');")
            # load next one if exists
            if i + 1 < len(videos):
                browser.execute_script("document.getElementById('" + otherPlayer + "').src = '" + videos[i + 1] + "';")
            
            # wait for the end of the video
            while True:
                if browser.execute_script("return document.getElementById('" + currentPlayer + "').ended;"):
                    break
            
            currentPlayer = otherPlayer
            i += 1
    

def mediaCB(data):
    for mediaData in data.medias.medias:
        mediaPath = mediaData.path
        media = medias[mediaPath]
        
        mediaPlayer.set_media(media)
        
        while mediaPlayer.get_position() < .998:
            mediaPlayer.set_rate(1.0)
            mediaPlayer.play()


def pathFeedbackCB(data):
    pass
    
    
if __name__ == '__main__':
    mediaPlayer = MediaPlayer()
    
    # ros node
    #rospy.init_node('media_player', log_level = rospy.INFO)
    #rospy.Subscriber('/robot01/scenario', ScenarioMsg, mediaCB)
    #rospy.spin()
