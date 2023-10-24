from RRTLib import RRTGraph, RRTMap 
from RRTLib import TreeNode
import numpy as np
import pygame as py

def main():
    dimensions = np.array([600, 1000])
    start = np.array([50, 50])
    goal = np.array([510, 510])
    obsdim = 30
    obsNum = 50 
    
    py.init()
    
    map = RRTMap(start,goal, dimensions, obsdim, obsNum)
    graph = RRTGraph(start, goal, 0, 0, dimensions, 30, 50)
    obst = graph.obstMake()
    map.drawOnMap(obst)
    
    py.display.update()
    py.event.clear()
    py.event.wait(0)
    
    
    
if __name__ == "__main__":
    main()