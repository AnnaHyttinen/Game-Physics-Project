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

A = (p1A, p2A, p3A, p1A) # this helps to draw lines from beginning to end
Ax, Ay = zip(*A) # creates lists of corner dots's x and y elements
cornersAx = [Ax] # do we really even need these? well, they are in use now
cornersAy = [Ay]

B = (p1B, p2B, p3B, p1B)
Bx, By = zip(*B)
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
    
    # an attempt to calculate and update the corners
    for i in Ax:
        i = i*math.cos(y) - i*math.sin(y)
    for i in Ay:
        i = i*math.sin(y) + i*math.cos(y)
    plt.plot(Ax, Ay)
    
    # Same for triangle B:
    
    if(dotsBy[-1] < 0 and VycmB < 0): # collision check for B
        VycmB = -e * VycmB
    else:
        VycmB = VycmB - g * dt
    
    cmBx = cmBx + VxcmB * dt
    cmBy = cmBy + VycmB * dt
    dotsBx.append(dotsBx[-1] + VxcmB * dt)
    dotsBy.append(dotsBy[-1] + VycmB * dt)
    
    for i in Bx:
        i = i*math.cos(y) - i*math.sin(y)
    for i in By:
        i = i*math.sin(y) + i*math.cos(y)
    plt.plot(Bx, By)
    
    # updating the other variables: 
    y = y + w * dt
    time += dt

# Plotting the lists and showing it
plt.plot(dotsAx, dotsAy, 'o')
plt.plot(dotsBx, dotsBy,'x')
plt.show()