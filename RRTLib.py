# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 07:41:28 2023

@author: 16187
"""
import pygame as py
import numpy as np
import math
import random
import numpy

class TreeNode():
    def __init__(self, locX, locY):
        self.locationX = locX
        self.locationY = locY
        self.children = []
        self.parent = None
        
class RRTMap():
    def __init__(self, start, goal, mapdimen, obsdim, obsNum):
        self.root = TreeNode(start[0], start[1])
        self.goal = TreeNode(goal[0], goal[1])
        self.goalXY = (self.goal.locationX, self.goal.locationY)
        self.startXY = (self.root.locationX, self.root.locationY)
        self.mapheight = mapdimen[0]
        self.mapwidth = mapdimen[1]
        
        #window display setting
        
        self.mapWinName = 'RRTPlanning Mqp'
        py.display.set_caption(self.mapWinName)
        self.map = py.display.set_mode((self.mapwidth, self.mapheight))
        self.map.fill((255, 255, 255))
        self.nodeRadius = 0
        self.nodeThicknes = 0
        self.edgeThickess = 1
        
        
        self.obstacles = []
        self.obsdim = obsdim
        self.obsNumber = obsNum
        
        #color
        self.grey = (70, 70, 70)
        self.blue = (0, 0, 250)
        self.green = (0, 250, 0)
        self.red = (250, 0, 0)
        self.white = (250, 250, 250)
        #we create 
        
    def drawObstacles(self, obstacles):
        obstalelist = obstacles.copy()
        while(len(obstalelist)>0):
            obstacles = obstalelist.pop(0) #TRICK two things happen the obstcle is the rec thet was deleted and used in next line of code
            py.draw.rect(self.map,self.grey, obstacles)
    def drawOnMap(self, obstacles):
        py.draw.circle(self.map, self.green, self.startXY, self.nodeRadius+5, 0)
        py.draw.circle(self.map, self.blue, self.goalXY,self.nodeRadius+20, 1) 
        self.drawObstacles(obstacles)     
        
        
class RRTGraph():
    def __init__(self, start, goal, numofIt, stepsize, mapdimen, obsdim, obsNum):
        self.root = TreeNode(start[0], start[1])
        self.startXY = (self.root.locationX, self.root.locationY)
        self.goal = TreeNode(goal[0], goal[1])
        self.mapheight = mapdimen[0]
        self.mapwidth = mapdimen[1]
        self.goalXY = (self.goal.locationX, self.goal.locationY)
        self.nearesNode = None
        self.Distance = 10000
        self.NumOfIteration = min(numofIt, 200)
        self.pathDistance = 0
        self.numwayPoints = 0
        self.rho = stepsize
        self.obstacles = []
        self.obsdim = obsdim
        self.obsNumber = obsNum
        self.pathDistance = 0
        self.numwayPoints = 0
        self.wayPoint = []
        self.obsdim = obsdim
        
    def rectRandomMake(self):
        #point that indicate the upper left coner of the screen
        obsPoint = np.array([0.0, 0.0])
        obsPoint[0] = int(random.uniform(0, self.mapwidth-self.obsdim))
        obsPoint[1] = int(random.uniform(0,self.mapheight - self.obsdim))
        return (obsPoint[0], obsPoint[1])
    
    def obstMake(self):
        obs = []
        
        for i in range(0, self.obsNumber):
            rectang = None
            rootgoalsh = True
            
            while rootgoalsh:
                upoint = self.rectRandomMake()
                rectang = py.Rect(upoint,(self.obsdim, self.obsdim))
                if rectang.collidepoint(self.startXY) or rectang.collidepoint(self.goalXY):
                    rootgoalsh = True
                else:
                    rootgoalsh = False
            obs.append(rectang) #this will save all the rect blaock
            
        self.obstacles = obs.copy() # we make a copy of the obstruction 
        
        return obs
    
    def unitVector(self, parent, childToBe):
        v = np.array([childToBe[0] - parent.locationX, childToBe[1] - parent.locationY])
        direction = v / np.linalg.norm(v)
        return direction

    def distanceBetween(self, root, point2):
        a = (root.locationX - point2[0]) ** 2
        b = (root.locationY - point2[1]) ** 2
        dist = np.sqrt(a + b)
        return dist
    def sampleAPoint(self):
        x = int(random.uniform(0, self.mapheight))
        y = int(random.uniform(0, self.mapwidth))
        point = np.array([x, y])
        return point
    
    def findNearest(self, root, point):
        if not root:
            return
        # Distance between the root and point is
        dist = self.distanceBetween(root, point)
        if dist <= self.Distance:
            self.NearestNode = root
            self.Distance = dist
        for child in root.children:
            self.findNearest(child, point)
            
    def steerToPoint(self, start, point1):
        offset = self.rho * self.unitVector(start, point1)
        point = np.array([start.locationX + offset[0], start.locationY + offset[1]])
        # When it exceeds the limit
        if point[0] >= self.mapheight:
            point[0] = self.mapheight - 1
        if point[1] >= self.mapwidth:
            point[1] = self.mapwidth - 1
        return point 
    
    def isItInObstacles(self, root, point):
        rectang = None
        vec = self.unitVector(root, point)
        testSample = np.array([0.0, 0.0])
        obs = self.obstacles.copy()
        for i in range(int(self.rho)):
            while len(obs)>0:
                rectang = obs.pop(0)
                testSample[0] = root.locationX + i * vec[0]
                testSample[1] = root.locationY + i * vec[1]

            # To check if the sample point lies within obstacles
                if rectang.collidepoint(testSample[0],testSample[1]):
                    return True
        return False 
    def addChild(self, locationX, locationY):
        # Base case
        if locationX == self.goal.locationX:
            self.NearestNode.children.append(self.goal)
            self.goal.parent = self.NearestNode
        else:
            tempNode = TreeNode(locationX, locationY)
            self.NearestNode.children.append(tempNode)
            tempNode.parent = self.NearestNode
    
    def goalFound(self, point):
        return self.distanceBetween(self.goal, point) < self.rho
    
    def retracePath(self, goal):
        if goal is None or goal.locationX == self.root.locationX:
            return
        self.numwayPoints += 1
        currentPos = np.array([goal.locationX, goal.locationY])
        self.wayPoint.insert(0, currentPos)
        self.pathDistance += self.rho
        self.retracePath(goal.parent)          