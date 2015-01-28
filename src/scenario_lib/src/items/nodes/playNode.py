from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode

class PlayNode(DiagramNode):
    nodeName = "Play"
    nodeCategory = ""
    
    maxInputs = 2
    minInputs = 2
    hasOutput = 0
    inputGroup = None
    
    def __init__(self, parent, canvas, position):
        super(PlayNode, self).__init__(parent, canvas, position)
    
    
    def output(self):
        super(PlayNode, self).output()
        
        return self.inputs[0].output()