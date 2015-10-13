import math

class Sequence():
    def __init__(self, timePosition, position, backward, focused = False):
        self.timePosition = timePosition
        self.position = position
        self.backward = backward
        self.focused = focused
        
    
    def save(self):
        result = {}
        result["timePosition"] = self.timePosition
        result["position"] = self.position
        result["backward"] = self.backward
        
        return result
    
    
    def load(self, data):
        self.timePosition = data["timePosition"]
        self.position = data["position"]
        self.backward = data["backward"]
    
    
    def getPositionPointIndex(self):
        return int(math.floor(self.position))
    