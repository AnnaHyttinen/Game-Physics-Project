# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import math

# edge points and centers of mass for triangles 
# A and B introduced in various ways: 

p1A = (2.0, 1.0)
p2A = (-1.0, 1.0)
p3A = (-1.0, -2.0)

cmA = (0.0, 0.0)
cmAx = 0.0 
cmAy = 0.0

p1B = (5.0, 3.0)
p2B = (3.0, 4.0)
p3B = (4.0, 2.0)

cmB = (4.0, 3.0)
cmBx = 4.0 
cmBy = 3.0

# general variables needed for calculations: 

time = 0.0
g = 9.81
dt = 0.1
y = 1.0 # greek alphabet that looks a bit like y
w = 1.0 # greek alphabet that looks a bit like w
e = 0.8

# introducing centers of mass for triangles A and B

VxcmA = 2.0
VycmA = 7.0
dotsAx = [cmAx] # Establishes a list of values
dotsAy = [cmAy]

VxcmB = -2.0
VycmB = 3.0
dotsBx = [cmBx]
dotsBy = [cmBy]

# wrapping up information about the triangles A and B

A = (p1A, p2A, p3A, p1A) # to draw lines from beginning to end
Ax = [p[0] for p in A]
Ay = [p[1] for p in A]
cornersAx = [Ax] # for the purposes of printing the triangles
cornersAy = [Ay]

B = (p1B, p2B, p3B, p1B)
Bx = [p[0] for p in B]
By = [p[1] for p in B]
cornersBx = [Bx]
cornersBy = [By]

# calculations for movement: 

while (time < 3.0):
    if(dotsAy[-1] < 0 and VycmA < 0): # collision check for A
        VycmA = -e * VycmA # turning the direction of force at floor
    else:
        VycmA = VycmA - g * dt # continuing the ordinary way
        
    # calculating and updating the center of mass
    cmAx = cmAx + VxcmA * dt
    cmAy = cmAy + VycmA * dt
    dotsAx.append(dotsAx[-1] + VxcmA * dt)
    dotsAy.append(dotsAy[-1] + VycmA * dt)
    
    # calculating and updating the corners
    """Ax[0] = Ax[0]*math.cos(y) - Ax[0]*math.sin(y)
    Ax[1] = Ax[1]*math.cos(y) - Ax[1]*math.sin(y)
    Ax[2] = Ax[2]*math.cos(y) - Ax[2]*math.sin(y)
    Ax[3] = Ax[0]

    Ay[0] = Ay[0]*math.sin(y) + Ay[0]*math.cos(y)
    Ay[1] = Ay[1]*math.sin(y) + Ay[1]*math.cos(y)
    Ay[2] = Ay[2]*math.sin(y) + Ay[2]*math.cos(y)
    Ax[3] = Ax[0]"""
    plt.plot(Ax, Ay)
    
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
    y = y + w * dt
    time += dt

# Plotting the lists and showing it
plt.plot(dotsAx, dotsAy, 'o')
plt.plot(dotsBx, dotsBy,'x')
plt.gca().set_aspect('equal')
plt.show()