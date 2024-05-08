# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import math

# reference points introduced

p1Ai = [0.2, 0.1]
p2Ai = [-0.1, 0.1]
p3Ai = [-0.1, -0.2]

p1Bi = [0.25, 0.0]
p2Bi = [-0.25, 0.25]
p3Bi = [0.0, -0.25]

cmi = (0.0, 0.0)
cmix = 0.0 
cmiy = 0.0

# starting points for centers of mass introduced (various ways)

cmA = (0.0, 2.0)
cmAx = 0.0 
cmAy = 2.0

cmB = (4.0, 3.0)
cmBx = 4.0 
cmBy = 3.0

# general variables needed for calculations

r = (0.0, 0.0, 0.0)
J = 0
time = 0.0
g = 9.81
dt = 0.03
yA = 1.0 # greek alphabet that looks a bit like y
yB = 1.5
wA = [0.0, 0.0, 1.75] # (omega)
wB = [0.0, 0.0, 1.5]
e = 0.8
m = 1.0 # just one mass, equal to both triangles
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
    dist = 0.0
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
            if (dist0 > dist):
                dist = dist0
                s = (i[0], i[1], 0.0)
    return s

previous = None
def collision_triangles(t1, t2, Vb, Va, B, A, wb, wa):
    global previous
    # going through all sides of t1 for collision with t2
    if (previous == None):
        previous = t1[0] # i in notes, a point of a side
    else:
        for i in t1: # i is another point of a side
            pre_x = previous[0]
            pre_y = previous[1]
            curr_x = i[0]
            curr_y = i[1]
            rii1 = ((curr_x-pre_x), (curr_y-pre_y), 0.0)
            for i in t2: # i is a point of other triangle
                xp = i[0]
                yp = i[1]
                rip = ((xp-pre_x), (yp-pre_y), 0.0)
                rii1xrip = cross_product(rii1, rip)
                if(rii1xrip[2] > 0): # if a point is inside the other polygon
                    # find the nearest polygon side i->i+1!
                    side = distance(xp, yp, t1)
                    
                    side2x = (side[0])*(side[0])
                    side2y = (side[1])*(side[1])
                    upper = cross_product(side, k)
                    lower = math.sqrt(side2x+side2y)
                    n_coll = ((upper[0]/lower), (upper[1]/lower), (upper[2]/lower))
                    
                    # point velocities of both polygons at P
                    rap = r_calculator(i, A)
                    rbp = r_calculator(i, B)
                    
                    Vap = Va + cross_product(wa, rap)
                    Vbp = Vb + cross_product(wb, rbp)
                    Vab = Vap - Vbp
                    Vabn = dot_product(Vab, n_coll)
                    
                    if (Vabn < 0):
                        return True
                        break

# calculations for movement: 

while (time < 1.0):
    # triangles colliding together detection
    if(collision_triangles(A, B, (VxcmA, VycmA), (VxcmB, VycmB), (cmAx, cmAy), 
                           (cmBx, cmBy), wA, wB)==True):
        # calculate impulse
        # update cm velocities
        # update angular velocities
        # positional update for both polygons
    if(collision_triangles(B, A, (VxcmB, VycmB), (VxcmA, VycmA), (cmBx, cmBy), 
                           (cmAx, cmAy), wB, wA)==True):
        # calculate impulse
        # update cm velocities
        # update angular velocities
        # positional update for both polygons
    
    # collision detection
    collision_A(A)
    if(A_collides == False):
        VycmA = VycmA - g * dt
    else:
        crossrn = cross_product(r, n)
        crossrn2 = crossrn[2]*crossrn[2]
        dotVpn = dot_product(Vp, n)
        J = -(1+e)*dotVpn/(((1/m)+crossrn2)/IA)
        VycmA += J/m
        VycmA = -VycmA
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
    
    # A gets updated for the collision detection
    A1 = (Ax[0], Ay[0])
    A2 = (Ax[1], Ay[1])
    A3 = (Ax[2], Ay[2])
    A = (A1, A2, A3, A1)
    
    # Same for triangle B
    
    collision_B(B)
    if(B_collides == False):
        VycmB = VycmB - g * dt
    else:
        crossrn = cross_product(r, n)
        crossrn2 = crossrn[2]*crossrn[2]
        dotVpn = dot_product(Vp, n)
        J = -(1+e)*dotVpn/(((1/m)+crossrn2)/IB)
        VycmB += J/m
        VycmB = -VycmB
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
    
    # B gets updated for the collision detection
    B1 = (Bx[0], By[0])
    B2 = (Bx[1], By[1])
    B3 = (Bx[2], By[2])
    B = (B1, B2, B3, B1)
    
    # updating values for time and y: 
    yA = yA + wA[2] * dt
    yB = yB + wB[2] * dt
    time += dt
    
    # The triangles collision attempt
    collision_triangles(A, B)

# Showing the result
plt.gca().set_aspect('equal')
plt.show()

    
    
    
    