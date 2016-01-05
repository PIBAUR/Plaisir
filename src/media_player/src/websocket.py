from simpleWebSocketServer.SimpleWebSocketServer import SimpleWebSocketServer, WebSocket

import rospy

class StreamingWebSocketServer(WebSocket):
    def __init__(self, server, sock, address, receivedMessageCallback):
        super(StreamingWebSocketServer, self).__init__(server, sock, address)
        
        self.receivedMessageCallback = receivedMessageCallback
    
    
    def play(self):
        self.sendMessage("play")
    
    
    def sendMedias(self, mediaPaths):
        self.sendMessage("medias:" + ";;;".join(mediaPaths))
    
    
    def sendRosTime(self, rosTime):
        self.sendMessage("rosTime:" + str(rosTime))

    
    def sendPause(self):
        self.sendMessage("pause")

    
    def sendRefresh(self):
        self.sendMessage("refresh")
        
        
    def executeScript(self, script):
        self.sendMessage("javascript:" + script)
    
    
    def handleConnected(self):
        rospy.loginfo("browser connected")
        
        
    def handleMessage(self):
        message = str(self.data)
        rospy.loginfo("received: " + message)
        self.receivedMessageCallback(message, self)


    def handleClose(self):
        rospy.loginfo("browser connection closed")