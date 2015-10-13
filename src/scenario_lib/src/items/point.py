class Point():
    def __init__(self, x = 0, y = 0, theta = 0):
        self._x = float(x)
        self._y = float(y)
        self._theta = float(theta)
    
    
    def x(self):
        return self._x
    
    
    def y(self):
        return self._y
    
    
    def theta(self):
        return self._theta
    
    
    def setX(self, x):
        self._x = float(x)
    
    
    def setY(self, y):
        self._y = float(y)
    
    
    def setTheta(self, theta):
        self._theta = float(theta)
    
    
    def isEqual(self, otherPoint):
        return self._x == otherPoint.x() and self._y == otherPoint.y()
    
    
    def clone(self):
        return Point(self._x, self._y, self._theta)