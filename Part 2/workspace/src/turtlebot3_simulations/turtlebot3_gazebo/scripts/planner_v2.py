#!/usr/bin/env python
"""
Created on Tue Mar 14 23:56:08 2023

@author: saksham
"""

import numpy as np
import time
import rospy
from geometry_msgs.msg import Twist
#%%

# inputs
# start = input("Enter start X and Y coordinates, and angle, separated by comma :")
# goal  = input("Enter goal X and Y coordinates, separated by comma :")

# start = tuple(map(int, start.split(",")))
# goal = tuple(map(int, goal.split(",")))

# start = (start[0]+50,start[1]+100,start[2])
# goal = (goal[0]+50,goal[1]+100)
start = (50,100,0)
goal = (550,50)

rpm = (15*0.1047198,30*0.1047198)

c = 5 + 10.5 # clearance + R (robot radius)
dt = 2

r = 3.25
L = 16.0

#%%
## Making obsticle map space for point p, clearence 5

# Rectangles
def obs1(p):
    if p[0] > (150-c) and p[0]<(165+c) and p[1]>(75-c):
        return False
    else:
        return True
def obs2(p):
    if p[0] > (250-c) and p[0]<(265+c) and p[1]<(125+c):
        return False
    else:
        return True  

# Circle
def obs3(p):
    if (400-p[0])**2 +  (110-p[1])**2 - (50+c)**2 < 0:
        return False
    else:
        return True 

#%%
# Checker if a node falls in the obstacle space
def checkFeasibility(node):
    if obs1(node) and obs2(node) and obs3(node):
        if node[0]>=c and node[0]<=600-c and node[1]>=c and node[1]<=200-c:
            return True
        else:
            return False
    else:
        return False
#%%
shifter = [(0,rpm[0]),(rpm[0],0),(rpm[0],rpm[0]),(0,rpm[1]),
           (rpm[1],0),(rpm[1],rpm[1]),(rpm[1],rpm[0]),(rpm[0],rpm[1])]

def costC(node,goal):
    d = np.sqrt((node[0]-goal[0])**2 + (node[1]-goal[1])**2)
    return d

nodeList = {}

def shift(node,cost):
    x,y,t = node
    t = np.deg2rad(t)
    for i in shifter:
        t_ = t + dt*r*(i[0]-i[1])/L
        if i[0] == i[1]:
            x_ = x + 0.5*(i[0]+i[1])*r*dt*np.cos(t)
            y_ = y + 0.5*(i[0]+i[1])*r*dt*np.sin(t)
        else:
            F = (L/2)*(i[0]+i[1])/(i[0]-i[1])
            x_ = x + F*( np.sin( (r*(i[0]-i[1])*dt/L) +t ) -np.sin(t))
            y_ = y - F*( np.cos( (r*(i[0]-i[1])*dt/L) +t ) -np.cos(t))
        childNode = (x_,y_,np.rad2deg(t_))
        if checkFeasibility(childNode):
            nodeList[childNode] = i
            yield childNode, cost+costC(node,childNode)
#%%
# main algorithm

def Djk(startState,goalState):
    
    # time calculation
    startTime = time.time()
    
    if not checkFeasibility(startState) or not checkFeasibility(goalState):
        print('Infeasable states! Check Input')
        return None
    
    closedNodes = {}
    openNodes = {startState:( costC(startState,goal) , costC(startState,goal) ,0,0,0)}
    # order is totalCost, cost2Goal, cost2come, parent, self
    nodeVisit = 255*np.ones((600,200))
    
    child = 1
    repeatSkip=0
    
    while True:
        
        # popping first node
        parent=list(openNodes.keys())[0]

        closedNodes[parent] = openNodes[parent]
        
        if costC(parent,goalState) < L/2:
            print("Goal Found after",len(closedNodes),"nodes in ",time.time()-startTime, " seconds!")
            print("overwrote nodes :",repeatSkip)
            break
            
        for node,cost in shift(parent,openNodes[parent][2]):
            
            if nodeVisit[round(node[0]),round(node[1])]==125:
                repeatSkip = repeatSkip +1
                pass
            
            else:
                if nodeVisit[round(node[0]),round(node[1])] == 255 and node != None:
                    # ...... and if not, add child
                    openNodes[node] = (1.5*costC(node,goalState) + cost,
                             costC(node,goalState),
                             cost,openNodes[parent][4],child)
                    child = child + 1
                    nodeVisit[round(node[0]),round(node[1])]=125
       
        nodeVisit[round(parent[0]),round(parent[1])] = 0
        del openNodes[parent]
        
        # Sort the dict before popping
        openNodes = dict(sorted(openNodes.items(), key=lambda x:x[1]))
    
    # backtracking
    backTrack = [node,parent]
    child = closedNodes[parent][3]
    while child >0:
        for key, value in closedNodes.items():
            
            if value[4] == child:
                node = key
                child = value[3]
                backTrack.append(node)
                
    backTrack.append(startState)
    backTrack.reverse()
    
    return backTrack,closedNodes,openNodes,nodeVisit
#%%


backTrack,closedNodes,openNodes,nodeVisit = Djk(start,goal)

#%%
rospy.init_node('backtrack_publisher')

# Create the publisher
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Set the rate at which to publish the values
rate = rospy.Rate(100) # 10Hz

value = 1

# Main loop
time_start = rospy.get_rostime()
while not rospy.is_shutdown():
    # Publish each value in the list
    if (rospy.get_rostime() - time_start) > rospy.Duration.from_sec(dt):
        value = value + 1
        time_start = rospy.get_rostime()
        if value > len(backTrack)-1:
            break
    twist_msg = Twist()
    twist_msg.linear.x = (r/2)*(nodeList[backTrack[value]][0] + nodeList[backTrack[value]][1])/100
    twist_msg.angular.z = (r/L)*(nodeList[backTrack[value]][0] - nodeList[backTrack[value]][1])
    print(twist_msg.linear.x,twist_msg.angular.z)
    
    # Publish the message
    pub.publish(twist_msg)
    
    # Wait for the specified time between each message
    rate.sleep()

twist_msg = Twist()
twist_msg.linear.x = 0
twist_msg.angular.z = 0
pub.publish(twist_msg)

print("Completed")
# %%
