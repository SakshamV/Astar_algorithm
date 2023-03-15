#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 14 23:56:08 2023

@author: saksham
"""

import numpy as np
import pygame
import time
from sys import exit

#%%
## Making obsticle map space for point p, clearence 5

# Rectangles
def obs1(p):
    if p[0] > (100-10) and p[0]<(150+10) and p[1]<(100+10):
        return False
    else:
        return True
def obs2(p):
    if p[0] > (100-10) and p[0]<(150+10) and p[1]>(150-10):
        return False
    else:
        return True  

# Hexagon
P_act = [[235,162.5],[300,200],[365,162.5],[365,87.5],[300,50],[235,87.5]]
P     = [[226.39,167.5],[300,210],[373.61,167.5],[373.61,82.5],[300,40],[226.39,82.5]]
L1 = np.polyfit([P[0][0],P[1][0]] , [P[0][1],P[1][1]] , 1)
L2 = np.polyfit([P[1][0],P[2][0]] , [P[1][1],P[2][1]] , 1)
L3 = np.polyfit([P[3][0],P[4][0]] , [P[3][1],P[4][1]] , 1)
L4 = np.polyfit([P[4][0],P[5][0]] , [P[4][1],P[5][1]] , 1)

def obs3(p):
    L1_ = p[1] - L1[0]*p[0] - L1[1] <0
    L2_ = p[1] - L2[0]*p[0] - L2[1] <0
    L3_ = p[1] - L3[0]*p[0] - L3[1] >0
    L4_ = p[1] - L4[0]*p[0] - L4[1] >0
    if L1_ and L2_ and L3_ and L4_ and p[0]>226.39 and p[0]<373.61:
        return False
    else:
        return True
    
# Triangle
Q_act = [[460,225],[510,125],[460,25]]
Q     = [[450,267.36],[521.18,125],[450,-17.36]]
Q1 = np.polyfit([Q[0][0],Q[1][0]] , [Q[0][1],Q[1][1]] , 1)
Q2 = np.polyfit([Q[1][0],Q[2][0]] , [Q[1][1],Q[2][1]] , 1)

def obs4(p):
    Q1_ = p[1] - Q1[0]*p[0] - Q1[1] <0
    Q2_ = p[1] - Q2[0]*p[0] - Q2[1] >0
    if Q1_ and Q2_ and p[0]>450 and 235>=p[1]>=15:
        return False
    else:
        return True
#%%
# Checker if a node falls in the obstacle space
def checkFeasibility(node):
    if obs1(node) and obs2(node) and obs3(node) and obs4(node):
        if node[0]>=10 and node[0]<=590 and node[1]>=10 and node[1]<=240:
            return True
        else:
            return False
    else:
        return False
#%%
shifter = [60,30,0,-30,-60]

def costC(node,goal):
    d = np.sqrt((node[0]-goal[0])**2 + (node[1]-goal[1])**2)
    return d

def shift(node,cost,step):
    for i in range(len(shifter)):
        childNode = (node[0] + step * np.cos(np.deg2rad( shifter[i] + node[2])) , 
                     node[1] + step * np.sin(np.deg2rad( shifter[i] + node[2])) , 
                     node[2] + shifter[i])
        if checkFeasibility(childNode):
            yield childNode, cost+1

#%%

# main algorithm

def Djk(startState,goalState,step):
    
    # time calculation
    startTime = time.time()
    
    if not checkFeasibility(startState) or not checkFeasibility(goalState):
        print('Infeasable states! Check Input')
        return None
    
    closedNodes = {}
    openNodes = {startState:( costC((0,0,0),goal) , costC((0,0,0),goal) ,0,0,0)}
    # order is totalCost, cost2Goal, cost2come, parent, self
    
    child = 1
    
    while True:
        
        # popping first node
        parent=list(openNodes.keys())[0]

        closedNodes[parent] = openNodes[parent]
        
        if costC(parent,goalState)<5:
            print("Goal Found after",len(closedNodes),"nodes in ",time.time()-startTime, " seconds!")
            break
            
        for node,cost in shift(parent,openNodes[parent][2],step):
            if node in openNodes:
                # If node in open nodes, update cost ........
                newCost = min(cost,openNodes[node][2])
                openNodes[node] = (newCost+openNodes[node][1] , openNodes[node][1],
                         newCost, openNodes[node][3],openNodes[node][4])
                pass
            else:
                if node not in closedNodes and node != None:
                    # ...... and if not, add child
                    openNodes[node] = (costC(node,goalState) + cost,
                             costC(node,goalState),
                             cost,openNodes[parent][4],child)
                    child = child + 1
       
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
    
    return backTrack,closedNodes,openNodes
#%%

# inputs
start = input("Enter start X and Y coordinates, and angle, separated by comma :")
goal  = input("Enter goal X and Y coordinates, and angle, separated by comma :")

start = tuple(map(int, start.split(",")))
goal = tuple(map(int, goal.split(",")))

step = 4

try:
    backTrack,closedNodes,openNodes = Djk(start,goal,step)
except:
    exit()

#%%
# Plotting and animation

pygame.init()
screen = pygame.display.set_mode([1200, 500])

imgID = 0

running = True
clock = pygame.time.Clock()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((255, 255, 255))

    pygame.draw.rect(screen, (0,0,0), pygame.Rect(100*2, 150*2, 50*2, 100*2)) #dist from left, top, w,h
    pygame.draw.rect(screen, (0,0,0), pygame.Rect(100*2, 0*2, 50*2, 100*2))
    pygame.draw.polygon(screen,(0,0,0), (np.array(Q_act)*2).tolist())
    pygame.draw.polygon(screen,(0,0,0), (np.array(P_act)*2).tolist())
    
    pygame.draw.rect(screen, (0,0,200), pygame.Rect(start[0]*2, 500-start[1]*2, 4*2, 4*2))
    pygame.draw.rect(screen, (0,0,200), pygame.Rect(goal[0]*2, 500-goal[1]*2, 4*2, 4*2))
    
#    for node in closedNodes:
#        for event in pygame.event.get():
#            if event.type == pygame.QUIT:
#                running = False
#                break
#        if not running: break
#    
#        pygame.draw.rect(screen, (0,100,0), pygame.Rect(node[0]*2, 500-node[1]*2, 1*2, 1*2))
#        clock.tick(4800)
#        pygame.display.update()
#        
#        name = 'Image'+str(imgID).zfill(8)+'.png'
#        if imgID%100 == 0:
#            #pygame.image.save(screen, name)
#            pass
#        imgID=imgID+1
        
    for node in backTrack:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break
        if not running: break
    
        pygame.draw.rect(screen, (200,0,0), pygame.Rect(node[0]*2, 500-node[1]*2, 1*2, 1*2))
        clock.tick(100)
        pygame.display.update()
        
        name = 'Image'+str(imgID).zfill(8)+'.png'
        if imgID%2 == 0:
            #pygame.image.save(screen, name)
            pass
        imgID=imgID+1
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
pygame.quit()


