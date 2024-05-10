class Triangle:
    def __init__(self, cm, points, angDispl, angVel, mass, I):
        self.cm = cm
        self.points = points
        self.angDispl = angDispl
        self.angVel = angVel
        self.mass = mass
        self.I = I
        print(f"angDispl = {angDispl}, angVel = {angVel}")
        for point in points:
            point.printPoint()
        cm.printPoint()
        
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def printPoint(self):
        print(f"Location: {self.x}, {self.y}")
    
class CenterOfMass(Point):
    def __init__(self,x,y, vx, vy):
        super().__init__(x,y)
        self.vx = vx
        self.vy = vy
    def printPoint(self):
        super().printPoint()
        print(f"Velocity: {self.vx}i + {self.vy}j")
