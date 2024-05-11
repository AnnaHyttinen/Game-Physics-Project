import matplotlib.pyplot as plt
import math

class Triangle:
    def __init__(self, cm, points, angDispl, angVel, mass, I):
        self.cm = cm
        self.points = points
        self.angDispl = angDispl
        self.angVel = angVel
        self.mass = mass
        self.I = I
        print(f"angDispl = {angDispl}, angVel = {angVel}")
        for point in self.points:
            point.printPoint()
        self.cm.printPoint()
    def updateCorners(self):
        for i in self.points:
            i.updatePoint(self.angDispl, self.cm)
    def updateCM(self, dt):
        self.cm.updatePoint(dt)
    def updateCMVelocity(self, g, dt):
        self.cm.updateNoCollisionVelocity(g, dt)
    def plotPoints(self):
        plt.plot([self.points[0].x,self.points[1].x,self.points[2].x,self.points[0].x,], [self.points[0].y,self.points[1].y,self.points[2].y,self.points[0].y,])
        #print([self.points[0].x,self.points[1].x,self.points[2].x,self.points[0].x,])
        #print([self.points[0].y,self.points[1].y,self.points[2].y,self.points[0].y,])
        
        
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def updatePoint(self, angDispl, cm):
        self.x = self.x*math.cos(angDispl) - self.y*math.sin(angDispl) + cm.x
        self.y = self.x*math.sin(angDispl) + self.y*math.cos(angDispl) + cm.y
    def printPoint(self):
        print(f"Location: {self.x}, {self.y}")
    def plotPoint(self):
        plt.plot(self.x, self.y)
    
class CenterOfMass(Point):
    def __init__(self,x,y, vx, vy):
        super().__init__(x,y)
        self.vx = vx
        self.vy = vy
    def printPoint(self):
        super().printPoint()
        print(f"Velocity: {self.vx}i + {self.vy}j")
    def updatePoint(self, dt):
        self.x = self.x + self.vx * dt
        self.y = self.y + self.vy * dt
        #self.printPoint()
    def updateNoCollisionVelocity(self, g, dt):
        self.vy = self.vy - g * dt