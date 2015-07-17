#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os

import rospy

from std_msgs.msg import Float64 as Float64Msg
from scenario_msgs.msg import Scenario as ScenarioMsg

import vlc
    
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
    # vlc init
    vlcInstance = vlc.Instance('--no-video-title-show')
    
    mediaPlayer = vlcInstance.media_player_new()
    mediaPlayer.toggle_fullscreen()
    
    # import medias
    mediaDirectory = "/home/artlab/Bureau/videos_notre_bon_plaisir/"
    mediaPaths = os.listdir(mediaDirectory)
    medias = {}
    for mediaPath in mediaPaths:
        if mediaPath.split(".")[-1] in ["mp4", "mov"]:
            mediaFullPath = os.path.join(mediaDirectory, mediaPath)
            medias[mediaFullPath] = vlcInstance.media_new('file://' + mediaFullPath)
    
    # ros node
    #rospy.init_node('media_player', log_level = rospy.INFO)
    #rospy.Subscriber('scenario', ScenarioMsg, mediaCB)
    #rospy.spin()
