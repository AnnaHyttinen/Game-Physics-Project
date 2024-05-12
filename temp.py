# -*- coding: utf-8 -*-

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
        #self.printInformation()
    def updateCorners(self):
        for point in self.points:
            point = point.updatePoint(self.angDispl, self.cm)
    def updateCM(self, dt):
        self.cm.updatePoint(dt)
    def updateCMVelocity(self, g, dt):
        self.cm.updateNoCollisionVelocity(g, dt)
    def getPointsX(self):
        return (self.points[0].x,self.points[1].x,self.points[2].x,self.points[0].x)
    def getPointsY(self):
        return (self.points[0].y,self.points[1].y,self.points[2].y,self.points[0].y)
    def printInformation(self):
        self.cm.printPoint()
        for point in self.points:
            point.printPoint()
    #def checkCollision(self, triangle):
        #collA = False
        #collB = False
        #if(abs(abs(self.cm.x)-abs(triangle.cm.x))<0.5 and abs(abs(self.cm.y)-abs(triangle.cm.y))<0.5):
            #collA = collision_triangles(self, triangle, (self.cm.vx, self.cm.vy), (triangle.cm.vx, triangle.cm.vy), (self.cm.x, self.cm.y), (triangle.cm.x, triangle.cm.y),self.angVel, triangle.angVel, self.mass,triangle.mass)
            #collA = collision_triangles(triangle,self, (triangle.cm.vx, triangle.cm.vy), (self.cm.vx, self.cm.vy), (triangle.cm.x, triangle.cm.y), (self.cm.x, self.cm.y), triangle.angVel, self.angVel, triangle.mass, self.mass)

class Point:
    def __init__(self, x, y,cmx, cmy):
        self.rx = x
        self.ry = y
        self.x = x + cmx
        self.y = y + cmy
    def updatePoint(self, angDispl, cm):
        newrx = self.rx*math.cos(angDispl) - self.ry*math.sin(angDispl)
        newry = self.rx*math.sin(angDispl) + self.ry*math.cos(angDispl)
        self.x = newrx + cm.x
        self.y = newry + cm.y
        self.rx = newrx
        self.ry = newry
        return self    
    def printPoint(self):
        print(f"Location: {self.x}, {self.y}")
    def plotPoint(self):
        plt.plot(self.x, self.y)
    
class CenterOfMass(Point):
    def __init__(self,x,y, vx, vy):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
    def printPoint(self):
        super().printPoint()
        print(f"Velocity: {self.vx}i + {self.vy}j")
    def updatePoint(self, dt):
        self.x = self.x + self.vx * dt
        self.y = self.y + self.vy * dt
        return self
    def updateNoCollisionVelocity(self, g, dt):
        self.vy = self.vy - g * dt

# reference points introduced, relative to cm

p1Ai = [0.2, 0.1]
p2Ai = [-0.1, 0.1]
p3Ai = [-0.1, -0.2]

p1Bi = [0.25, 0.0]
p2Bi = [-0.25, 0.25]
p3Bi = [0.0, -0.25]

# starting points for centers of mass introduced (various ways)

cmAx = 0.0 
cmAy = 2.0
cmA = (cmAx, cmAy)

cmBx = 4.0 
cmBy = 3.0
cmB = (cmBx, cmBy)

# general variables needed for calculations

r = (0.0, 0.0, 0.0)
J = 0
time = 0.0
g = 9.81
dt = 0.02
yA = 1.0 # greek alphabet that looks a bit like y
yB = 1.5
wA = [0.0, 0.0, 1.75] # (omega)
wB = [0.0, 0.0, 1.5]
e = 0.8
mA = 1.0
mB = 1.0
n = (0.0, 1.0, 0.0)
IA = 0.15
IB = 0.125
k = [0.0, 0.0, 1.0]

# introducing components of velocity and tracks centers of masses

VxcmA = 2.0
VycmA = 4.0
dotsAx = [cmAx] # Establishes a list of values
dotsAy = [cmAy]

VxcmB = -2.0
VycmB = 3.0
dotsBx = [cmBx]
dotsBy = [cmBy]

# setting up the triangles A and B

p1A = (p1Ai[0] + cmA[0], p1Ai[1] + cmA[1])
p2A = (p2Ai[0] + cmA[0], p2Ai[1] + cmA[1])
p3A = (p3Ai[0] + cmA[0], p3Ai[1] + cmA[1])

A = (p1A, p2A, p3A, p1A) # to draw lines from beginning to end
Ax = [p[0] for p in A]
Ay = [p[1] for p in A]
cornersAx = [Ax] # for the purposes of printing the triangles
cornersAy = [Ay]

p1B = (p1Bi[0] + cmB[0], p1Bi[1] + cmB[1])
p2B = (p2Ai[0] + cmB[0], p2Bi[1] + cmB[1])
p3B = (p3Bi[0] + cmB[0], p3Bi[1] + cmB[1])

B = (p1B, p2B, p3B, p1B)
Bx = [p[0] for p in B]
By = [p[1] for p in B]
cornersBx = [Bx]
cornersBy = [By]

CCm = CenterOfMass(0.0, 2.0, 2.0, 4.0)
C1 = Point(0.2, 0.3, CCm.x, CCm.y)
C2 = Point(-0.1, 0.1, CCm.x, CCm.y)
C3 = Point(-0.1, -0.2, CCm.x, CCm.y)
C = Triangle(CCm, [C1, C2, C3], yA, wA, mA, IA) #copy of A

DCm = CenterOfMass(4.0,3.0, -2.0,3.0)
D1 = Point(0.25, 0.0, DCm.x, DCm.y)
D2 = Point(-0.25, 0.25, DCm.x, DCm.y)
D3 = Point(0, -0.25, DCm.x, DCm.y)
D = Triangle(DCm, [D1, D2, D3], yB, wB, mB, IB) #copy of B

# Inserting functions

def dot_product(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]

def cross_product(v1, v2):
    return [
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0]
    ]

# Calculate r for point p
def r_calculator(p, cm):
    a = p[0]-cm[0]
    b = p[1]-cm[1]
    c = 0.0
    return (a, b, c)
    
# Collision detection
A_collides = False
B_collides = False
Vp = None
r = None
def collision_A(triangle):
    global wA
    global VxcmA
    global VycmA
    global A_collides
    global Vp
    global r
    for i in triangle:
        r = r_calculator(i, cmA)
        wr = cross_product(wA, r)
        Vp = (VxcmA - wr[0], VycmA - wr[1], 0.0)
        if (Vp[1] < 0.0 and i[1] < 0.0):
            A_collides = True
            break
        else:
            A_collides = False
            
def collision_B(triangle):
    global wB
    global VxcmB
    global VycmB
    global B_collides
    global Vp
    global r
    for i in triangle:
        r = r_calculator(i, cmB)
        wr = cross_product(wB, r)
        Vp = (VxcmB - wr[0], VycmB - wr[1], 0.0)
        if (Vp[1] < 0.0 and i[1] < 0.0):
            B_collides = True
            break
        else:
            B_collides = False

def distance(x, y, t): # triangle collision system uses this to find a distance
    dist = 100
    s = None
    previ = None
    for i in t:
        if (previ == None):
            previ = i
        else:
            up = (i[0]-previ[0])*(y-previ[1]) - (x-previ[0])*(i[1]-previ[1])
            first2 = (i[0]-previ[0])*(i[0]-previ[0])
            second2 = (i[1]-previ[1])*(i[1]-previ[1])
            down = math.sqrt(first2+second2)
            if(down == 0):
                down = 0.0000000001
            dist0 = abs(up/down)
            if (dist0 < dist):
                dist = dist0
                s = (i[0], i[1], 0.0)
    return s

previousA = None
previousB = None
in_count = 0
count = 0

def collision_triangles(t1, t2, Vb, Va, cB, cA, wb, wa, mb, ma):
    global previousA
    global previousB
    global wA
    global wB
    global VxcmA
    global VycmA
    global VxcmB
    global VycmB
    global A
    global in_count
    global count
    
    # going through all points of t2 against the walls of t1
    
    for i in t2: # i is a point of a triangle
        count += 1
        if(count >= 3):
            count = 0
            break
        
        xp = i[0]
        yp = i[1]
        pre_x = None
        pre_y = None
        curr_x = None
        curr_y = None
        
        if (previousA == None or previousB == None):
            if (t1 == A):
                previousA = t1[0] # holds the first point of a side
            else:
                previousB = t1[0]

        else:
            for i in t1: # holds the second point of a side
                if (t1 == A):
                    pre_x = previousA[0]
                    pre_y = previousA[1]
                    previousA = i
                else:
                    pre_x = previousB[0]
                    pre_y = previousB[1]
                    previousB = i
                curr_x = i[0]
                curr_y = i[1]
        
        rii1xrip = None
        
        if (pre_x != None and pre_y != None):           
            rip = ((xp-pre_x), (yp-pre_y), 0.0) # point compared to first point of side
            rii1 = ((curr_x-pre_x), (curr_y-pre_y), 0.0) # defines a side
            rii1xrip = cross_product(rii1, rip)

        if(rii1xrip != None and rii1xrip[2] > 0): 
            in_count += 1
            
        if (in_count == 3):
            # point is inside the other polygon
            # finding the nearest polygon side i->i+1
            for i in t2:
                xp = i[0]
                yp = i[1]
                side = distance(xp, yp, t1)
                    
                side2x = (side[0])*(side[0])
                side2y = (side[1])*(side[1])
                upper = cross_product(side, k)
                lower = math.sqrt(side2x+side2y)
                n_coll = ((upper[0]/lower), (upper[1]/lower), (upper[2]/lower))
                    
                # point velocities of both polygons at P
                rap = r_calculator(i, cA)
                rbp = r_calculator(i, cB)

                Vap = (Va[0] + cross_product(wa, rap)[0])+(Va[1] + cross_product(wa, rap)[1])
                Vbp = (Vb[0] + cross_product(wb, rbp)[0])+(Vb[1] + cross_product(wb, rbp)[1])
                Vab = ((Vap - Vbp), 0.0, 0.0)
                Vabn = dot_product(Vab, n_coll)
                    
                if (Vabn < 0):
                    print("collision detected")
                    # calculate impulse
                    rapn2 = (cross_product(rap, n_coll)[2])*(cross_product(rap, n_coll)[2])
                    J = -(1+e)*(dot_product(Vab, n_coll))/((1/ma)+(1/mb)+(rapn2/IA))
                            
                    Vaf = ((Va[0] + J/ma * n_coll[0]),(Va[1]) + J/ma * n_coll[1])
                    Vbf = ((Vb[0] + J/mb * n_coll[0]),(Vb[1] + J/mb * n_coll[1]))
                            
                    crosstempA = wa[2] - J/IA * cross_product(rap, n_coll)[2]
                    crosstempB = wb[2] + J/IB * cross_product(rbp, n_coll)[2]
                    waf = (0.0, 0.0, crosstempA)
                    wbf = (0.0, 0.0, crosstempB)
                            
                    if(t1 == A):
                        VxcmA = Vaf[0]
                        VycmA = Vaf[1]
                        VxcmB = Vbf[0]
                        VycmB = Vbf[1]
                        wA = wbf
                        wB = waf
                    else:
                        VxcmA = Vbf[0]
                        VycmA = Vbf[1]
                        VxcmB = Vaf[0]
                        VycmB = Vaf[1]
                        wA = waf
                        wB = wbf                            
                            
                        # updated cm velocities
                        # updated angular velocities
                        # positional update for both polygons: later in the code

                    return True
                    break
    return False

# calculations for movement: 

while (time < 3.0):
    collA = False
    collB = False
    # triangle collision detection only if the triangles are near to each other
    if(abs(abs(cmAx)-abs(cmBx))<0.5 and abs(abs(cmAy)-abs(cmBy))<0.5):
        VA = (VxcmA, VycmA)
        VB = (VxcmB, VycmB)
        CMA = (cmAx, cmAy)
        CMB = (cmBx, cmBy)
        collA = collision_triangles(A, B, VA, VB, cmA, cmB, wA, wB, mA, mB)
        collB = collision_triangles(B, A, VB, VA, cmB, cmA, wB, wA, mB, mA)

    # ground collision detection only if the previous collisions did not happen
    if(collA!=True and collB!=True):
        collision_A(A)
        if(A_collides == False):
            VycmA = VycmA - g * dt
        else:
            crossrn = cross_product(r, n)
            crossrn2 = crossrn[2]*crossrn[2]
            dotVpn = dot_product(Vp, n)
            J = -(1+e)*dotVpn/(((1/mA)+crossrn2)/IA)
            VycmA += J/mA
            VycmA = -VycmA
            wA = list(wA)
            wA[2] += J/IA * crossrn[2]

    # calculating and updating the center of mass
    cmAx = cmAx + VxcmA * dt
    cmAy = cmAy + VycmA * dt
    dotsAx.append(dotsAx[-1] + VxcmA * dt)
    dotsAy.append(dotsAy[-1] + VycmA * dt)
    
    
    # calculating and updating the corners
    Ax[0] = p1Ai[0]*math.cos(yA) - p1Ai[1]*math.sin(yA) + cmAx
    Ax[1] = p2Ai[0]*math.cos(yA) - p2Ai[1]*math.sin(yA) + cmAx
    Ax[2] = p3Ai[0]*math.cos(yA) - p3Ai[1]*math.sin(yA) + cmAx
    Ax[3] = Ax[0]
    
    Ay[0] = p1Ai[0]*math.sin(yA) + p1Ai[1]*math.cos(yA) + cmAy
    Ay[1] = p2Ai[0]*math.sin(yA) + p2Ai[1]*math.cos(yA) + cmAy
    Ay[2] = p3Ai[0]*math.sin(yA) + p3Ai[1]*math.cos(yA) + cmAy
    Ay[3] = Ay[0]
    
    
    plt.plot(Ax, Ay)
    
    # A gets updated
    A1 = (Ax[0], Ay[0])
    A2 = (Ax[1], Ay[1])
    A3 = (Ax[2], Ay[2])
    A = (A1, A2, A3, A1)
    

    #Triangle class triangles    
    C.updateCMVelocity(g, dt)
    C.updateCM(dt)
    C.updateCorners()
    CX = C.getPointsX()
    CY = C.getPointsY()
    #plt.plot(CX,CY) #commented out for clarity
    
    D.updateCMVelocity(g, dt)
    D.updateCM(dt)
    D.updateCorners()
    DX = D.getPointsX()
    DY = D.getPointsY()
    #plt.plot(DX,DY) #commented out for clarity
    
    # Same for triangle B
    if(collA!=True and collB!=True):
        collision_B(B)
        if(B_collides == False):
            VycmB = VycmB - g * dt
        else:
            crossrn = cross_product(r, n)
            crossrn2 = crossrn[2]*crossrn[2]
            dotVpn = dot_product(Vp, n)
            J = -(1+e)*dotVpn/(((1/mB)+crossrn2)/IB)
            VycmB += J/mB
            VycmB = -VycmB
            wB = list(wB)
            wB[2] += J/IB * crossrn[2]
            
    # calculating and updating the center of mass
    cmBx = cmBx + VxcmB * dt
    cmBy = cmBy + VycmB * dt
    dotsBx.append(dotsBx[-1] + VxcmB * dt)
    dotsBy.append(dotsBy[-1] + VycmB * dt)
    
    # calculating and updating the corners
    Bx[0] = p1Bi[0]*math.cos(yB) - p1Bi[1]*math.sin(yB) + cmBx
    Bx[1] = p2Bi[0]*math.cos(yB) - p2Bi[1]*math.sin(yB) + cmBx
    Bx[2] = p3Bi[0]*math.cos(yB) - p3Bi[1]*math.sin(yB) + cmBx
    Bx[3] = Bx[0]
    
    By[0] = p1Bi[0]*math.sin(yB) + p1Bi[1]*math.cos(yB) + cmBy
    By[1] = p2Bi[0]*math.sin(yB) + p2Bi[1]*math.cos(yB) + cmBy
    By[2] = p3Bi[0]*math.sin(yB) + p3Bi[1]*math.cos(yB) + cmBy
    By[3] = By[0]
    
    plt.plot(Bx, By)
    
    # B gets updated
    B1 = (Bx[0], By[0])
    B2 = (Bx[1], By[1])
    B3 = (Bx[2], By[2])
    B = (B1, B2, B3, B1)
    
    # updating the rest of the values: 
    yA = yA + wA[2] * dt
    yB = yB + wB[2] * dt
    time += dt

# Showing the result
plt.gca().set_aspect('equal')
plt.show()