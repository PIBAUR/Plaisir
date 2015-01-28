from src.scenario_lib.src.items.nodes.diagramNode import DiagramNode

class ScenarioNode(DiagramNode):
    nodeName = "Scenario"
    nodeCategory = ""
    
    maxInputs = 0
    minInputs = 0
    hasOutput = 1
    inputGroup = None
    
    def __init__(self, parent, canvas, position):
        super(ScenarioNode, self).__init__(parent, canvas, position)
    
    
    def output(self):
        super(ScenarioNode, self).output()
        
        return self.inputs[0].output()