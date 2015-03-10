#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import time

import wx

import pygst
import gst
import gobject
gobject.threads_init()


class MediaPlayer(wx.App):
    def __init__(self, endedCallback):
        super(MediaPlayer, self).__init__()
        
        self.endedCallback = endedCallback
        
        self.duration = None
        self.position = 0
        self.requestedRate = 3.0
        self.rate = self.requestedRate
        
    
    def OnInit(self):
        # ui
        window = wx.Frame(None)
        window.Bind(wx.EVT_CLOSE,self.destroy)
        self.movie_window = wx.Panel(window)
        #window.ShowFullScreen(True)
        window.Show()
        self.SetTopWindow(window)
        
        # player
        self.player = gst.element_factory_make("playbin", "player")
        bus = self.player.get_bus()
        bus.add_signal_watch()
        bus.enable_sync_message_emission()
        bus.connect('message', self.handleMessage)
        bus.connect('sync-message::element', self.handleSyncMessage)
        
        return True
    
    
    def start(self, filePath):
        self.now = time.time()
        self.player.set_property('uri',"file://" + filePath)
        self.player.set_state(gst.STATE_PLAYING)
    
    
    def handleMessage(self, bus, message):
        return
        if message.type == gst.MESSAGE_ASYNC_DONE:
            if self.duration is None:
                print time.time() - self.now
                # start
                self.player.seek(self.requestedRate, gst.FORMAT_TIME, gst.SEEK_FLAG_FLUSH, gst.SEEK_TYPE_SET, self.position, gst.SEEK_TYPE_NONE, -1)
                self.rate = self.requestedRate
                self.player.set_state(gst.STATE_PLAYING)
                self.duration = self.player.query_duration (gst.FORMAT_TIME, None)[0]
        elif message.type == gst.MESSAGE_EOS:
            # stop
            self.player.set_state(gst.STATE_NULL)
            self.duration = None
            self.position = None
            self.rate = self.requestedRate
            self.endedCallback()
        elif message.type == gst.MESSAGE_ERROR:
            self.player.set_state(gst.STATE_NULL)
        else:
            # while playing
            try:
                self.position = self.player.query_position(gst.FORMAT_TIME, None)[0]
                if self.rate != self.requestedRate:
                    self.player.seek(self.requestedRate, gst.FORMAT_TIME, gst.SEEK_FLAG_FLUSH, gst.SEEK_TYPE_SET, self.position, gst.SEEK_TYPE_NONE, -1)
                    self.rate = self.requestedRate
            except gst.QueryError:
                pass
   
   
    def handleSyncMessage(self, bus, message):
        if message.structure is None:
            return
        message_name = message.structure.get_name()
        if message_name == 'prepare-xwindow-id':
            imagesink = message.src
            imagesink.set_property('force-aspect-ratio', True)
            imagesink.set_xwindow_id(self.movie_window.GetHandle())
    
    
    def destroy(self,event):
        #Stop the player pipeline to prevent a X Window System error
        self.player.set_state(gst.STATE_NULL)
        event.Skip()


def playNextVideo():
    mediaPlayer.start(os.path.expanduser("~") + "/video.mp4")

mediaPlayer = MediaPlayer(playNextVideo)
playNextVideo()
mediaPlayer.MainLoop()