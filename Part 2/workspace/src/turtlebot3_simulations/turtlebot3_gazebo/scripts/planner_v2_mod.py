#!/usr/bin/env python
"""
Created on Tue Mar 14 23:56:08 2023

@author: saksham
"""

import numpy as np
import time
import rospy
from geometry_msgs.msg import Twist
import pygame
#%%

# inputs
# start = input("Enter start X and Y coordinates, and angle, separated by comma :")
# goal  = input("Enter goal X and Y coordinates, separated by comma :")

# start = tuple(map(int, start.split(",")))
# goal = tuple(map(int, goal.split(",")))

# start = (start[0]+50,start[1]+100,start[2])
# goal = (goal[0]+50,goal[1]+100)
start = (50,100,0)
goal = (550,100)

rpm = (10*0.1047,20*0.1047)
# rpm = (2.80,2.80)

c = 15 + 10.5 # clearance + R (robot radius)
dt = 5.5

r = 3.3
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
        childNode = (x_,y_,np.arctan2(np.sin(np.rad2deg(t_)),np.cos(np.rad2deg(t_))) + np.pi )

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
    nodeVisit = 255*np.ones((6000,2000,18))
    
    child = 1
    repeatSkip=0

    print("initiated")
    
    while True:
        
        # popping first node
        parent=list(openNodes.keys())[0]

        closedNodes[parent] = openNodes[parent]
        
        if costC(parent,goalState) < L/2:
            print("Goal Found after",len(closedNodes),"nodes in ",time.time()-startTime, " seconds!")
            print("overwrote nodes :",repeatSkip)
            break
            
        for node,cost in shift(parent,openNodes[parent][2]):
            
            if nodeVisit[int(round(10*node[0])),int(round(10*node[1])),int(round(0.05*node[2]))]==125:
                repeatSkip = repeatSkip +1
                pass
            
            else:
                if nodeVisit[int(round(10*node[0])),int(round(10*node[1])),int(round(0.05*node[2]))] == 255 and node != None:
                    # ...... and if not, add child
                    openNodes[node] = (2.5*costC(node,goalState) + cost,
                             costC(node,goalState),
                             cost,openNodes[parent][4],child)
                    child = child + 1
                    nodeVisit[int(round(10*node[0])),int(round(10*node[1])),int(round(0.05*node[2]))]=125
       
        nodeVisit[int(round(10*parent[0])),int(round(10*parent[1])),int(round(0.05*parent[1]))] = 0
        del openNodes[parent]
        
        # Sort the dict before popping
        openNodes = dict(sorted(openNodes.items(), key=lambda x:x[1]))
    
    print("\nDone Exploring\n")
    
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

pygame.init()
screen = pygame.display.set_mode([1800, 600])

imgID = 0

running = True
clock = pygame.time.Clock()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((255, 255, 255))

    pygame.draw.rect(screen, (0,0,0), pygame.Rect(150*3, 0*3, 15*3, 125*3)) #dist from left, top, w,h
    pygame.draw.rect(screen, (0,0,0), pygame.Rect(250*3, 75*3, 15*3, 125*3))
    pygame.draw.circle(screen, (0,0,0), (400*3,600-110*3) , 50*3)
    
    pygame.draw.rect(screen, (0,0,200), pygame.Rect(start[0]*3, 600-start[1]*3, 4*3, 4*3))
    pygame.draw.rect(screen, (0,0,200), pygame.Rect(goal[0]*3, 600-goal[1]*3, 4*3, 4*3))
        
    for i in range(len(backTrack)-2):
        node=backTrack[i]
        nodeN = backTrack[i+1]
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break
        if not running: break
        
        pygame.draw.line(screen, (200,0,0), (3*node[0],600-3*node[1]),  (3*nodeN[0],600-3*nodeN[1]), width=2)
        pygame.draw.circle(screen, (200,0,0), (3*nodeN[0],600-3*nodeN[1]), 3, width=0)
        clock.tick(50)
        pygame.display.update()
        
        name = 'Image'+str(imgID).zfill(8)+'.png'
        if imgID%1 == 0:
            #pygame.image.save(screen, name)
            pass
        imgID=imgID+1
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
pygame.quit()

# %%
backTrack.reverse()
backTrack.pop()
#backTrack.append(backTrack[-1])
backTrack.reverse()
#%%
# rospy.init_node('backtrack_publisher')

# # Create the publisher
# pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# # Set the rate at which to publish the values
# # rate = rospy.Rate(1/dt) # 10Hz
# rate = rospy.Rate(100)

# value = 0

# # Main loop
# time_start = rospy.get_rostime()

# counter = 0

# while True:
#     #Publish each value in the list
#     if (rospy.get_rostime() - time_start) > rospy.Duration.from_sec(dt):
#         value = value + 1
#         time_start = rospy.get_rostime()
#         if value > len(backTrack)-1:
#             break
#     twist_msg = Twist()
#     twist_msg.linear.x = (r/2)*(nodeList[backTrack[value]][0] + nodeList[backTrack[value]][1])/100
#     twist_msg.angular.z = (r/L)*(nodeList[backTrack[value]][0] - nodeList[backTrack[value]][1])
    
#     # Publish the message
#     pub.publish(twist_msg)
#     print(value, " : ", twist_msg.linear.x, twist_msg.angular.z)
    
#     # Wait for the specified time between each message
#     rate.sleep()
#     # value = value + 1
    
#     # if value > len(backTrack)-1:
#     #         break

# twist_msg = Twist()
# twist_msg.linear.x = 0
# twist_msg.angular.z = 0
# pub.publish(twist_msg)

# print("Completed")
# %%
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import tf

# Set up variables
index = 0

# Callback function for the '/gazebo/model_states' topic subscriber
def model_states_cb(msg):
    global index

    # Store the latest model states message
    position = msg.pose[3].position
    orientation = msg.pose[3].orientation

    goal_temp = (backTrack[index][0]*0.01 -0.5 , backTrack[index][1]*0.01 - 1)
    rotation_matrix = tf.transformations.quaternion_matrix([orientation.x,orientation.y,orientation.z,orientation.w])
    rpy = tf.transformations.euler_from_matrix(rotation_matrix,'sxyz')


    message = Twist()
    message.linear.x = 0.09
    message.angular.z = -0.5 * (rpy[2] - np.arctan2(goal_temp[1]-position.y,goal_temp[0]-position.x))

    if ((position.x - goal_temp[0])**2 + (position.y - goal_temp[1])**2) < 0.1:
        index = index + 1
        print("temp goal reached!")
        return

    # Extract the current pose of the turtlebot
    cmd_vel_pub.publish(message)

# ROS node initialization
rospy.init_node('turtlebot_backtrack')

# Set up subscribers and publishers
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

while not rospy.is_shutdown():
    model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_cb)
    rospy.spin()
