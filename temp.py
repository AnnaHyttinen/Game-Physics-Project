# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
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
dt = 0.1
y = 1.0 # greek alphabet that looks a bit like y
w = [0.0, 0.0, 1.0] # (omega)
e = 0.8
m = 1.0 # just one mass, equal to both triangles
n = (0.0, 1.0, 0.0)
IA = 0.15
IB = 0.125

# introducing components of velocity and tracks centers of masses

VxcmA = 2.0
VycmA = 7.0
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
def collision_A(triangle):
    global w
    global VycmA
    for i in triangle:
        print(i)
        r = r_calculator(i, cmA)
        wr = cross_product(w, r)
        Vp = (VxcmA - wr[0], VycmA - wr[1], 0.0)
        if (Vp[1] >= 0.0 or i[1] > 0.0):
            VycmA = VycmA - g * dt
        else:
            crossrn = cross_product(r, n)
            crossrn2 = crossrn[2]*crossrn[2]
            dotVpn = dot_product(Vp, n)
            J = -(1+e)*dotVpn/(((1/m)+crossrn2)/IA)
            VycmA += J/m
            w[2] += J/IA * crossrn[2]
            return
    
# calculations for movement: 

while (time < 3.0):
    collision_A(A)
        
    # calculating and updating the center of mass
    cmAx = cmAx + VxcmA * dt
    cmAy = cmAy + VycmA * dt
    dotsAx.append(dotsAx[-1] + VxcmA * dt)
    dotsAy.append(dotsAy[-1] + VycmA * dt)
    
    # calculating and updating the corners
    Ax[0] = p1Ai[0]*math.cos(y) - p1Ai[1]*math.sin(y) + cmAx
    Ax[1] = p2Ai[0]*math.cos(y) - p2Ai[1]*math.sin(y) + cmAx
    Ax[2] = p3Ai[0]*math.cos(y) - p3Ai[1]*math.sin(y) + cmAx
    Ax[3] = Ax[0]
    
    Ay[0] = p1Ai[0]*math.sin(y) + p1Ai[1]*math.cos(y) + cmAy
    Ay[1] = p2Ai[0]*math.sin(y) + p2Ai[1]*math.cos(y) + cmAy
    Ay[2] = p3Ai[0]*math.sin(y) + p3Ai[1]*math.cos(y) + cmAy
    Ay[3] = Ay[0]
    
    plt.plot(Ax, Ay)
    
    # A gets updated for the collision detection
    A1 = (Ax[0], Ay[0])
    A2 = (Ax[1], Ay[1])
    A3 = (Ax[2], Ay[2])
    A = (A1, A2, A3, A1)
    
    # Same for triangle B
    
    if(dotsBy[-1] < 0 and VycmB < 0): # collision check for B
        VycmB = -e * VycmB
    else:
        VycmB = VycmB - g * dt
    
    cmBx = cmBx + VxcmB * dt
    cmBy = cmBy + VycmB * dt
    dotsBx.append(dotsBx[-1] + VxcmB * dt)
    dotsBy.append(dotsBy[-1] + VycmB * dt)

    """Bx[0] = Bx[0]*math.cos(y) - Bx[0]*math.sin(y)
    Bx[1] = Bx[1]*math.cos(y) - Bx[1]*math.sin(y)
    Bx[2] = Bx[2]*math.cos(y) - Bx[2]*math.sin(y)
    Bx[3] = Bx[0]

    By[0] = By[0]*math.sin(y) + By[0]*math.cos(y)
    By[1] = By[1]*math.sin(y) + By[1]*math.cos(y)
    By[2] = By[2]*math.sin(y) + By[2]*math.cos(y)
    Bx[3] = Bx[0]"""
    plt.plot(Bx, By)
    
    # updating the other variables: 
    y = y + w[2] * dt
    time += dt

# Plotting the lists and showing it
plt.plot(dotsAx, dotsAy, 'o')
plt.plot(dotsBx, dotsBy,'x')
plt.gca().set_aspect('equal')
plt.show()

    
    
    
    