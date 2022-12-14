#!/usr/bin/env python

import heapq
import math
import time
import numpy as np


obstacle = 1
grid = []

def draw_grid(grid):
    for i in grid: 
        for j in i:
            if j == obstacle:
                print ("###", end=" ")
            else:
                print ("{:<3}".format(j), end=" ") 
        print("")

def is_blocked(curr_node, direction, grid):
    if curr_node[0] + direction[0] < 0 or curr_node[0] + direction[0] >= len(grid):
        return True
    if curr_node[1] + direction[1] < 0 or curr_node[1] + direction[1] >= len(grid[1]):
        return True
    if direction[0] != 0 and direction[1] != 0 :
        if grid[curr_node[0] + direction[0]][curr_node[1]] == obstacle or grid[curr_node[1]][curr_node[1] + direction[1]] == obstacle :
            return True
        if grid[curr_node[0] + direction[0]][curr_node[1] + direction[1]] == obstacle :
            return True
    else:
        if direction[0] != 0:
            if grid[curr_node[0] + direction[0]][curr_node[1]] == obstacle:
                return True
        else :
            if grid[curr_node[0]][curr_node[1] + direction[1]] == obstacle:
                return True
    return False


def path_lenght(path):
    total_dist = 0

    for wp in range(len(path)-1):
        a = np.array(path[wp])
        b = np.array(path[wp + 1])

        distance = np.linalg.norm(b - a)

        total_dist = total_dist + distance
    
    return total_dist


def djikstra(grid, start, goal):
    closed_list = set()
    came_from = {}
    gscore = {start: 0 }
    fscore = {start:0}

    starttime = time.time()
    open_list = []
    heapq.heappush(open_list,(fscore[start],start))

    while open_list:
        curr_node = heapq.heappop(open_list)[1]
        if curr_node == goal:
            path = []
            while curr_node in came_from:
                path.append(curr_node)
                curr_node = came_from[curr_node]
            path.append(start)
            path.reverse()
            endtime = time.time()
            time_running = round(endtime - starttime, 5)
            return (path, time_running)
        
        closed_list.add(curr_node)
        for dir in [(0, 1), # kanan
            (0, -1), # kiri
            (1, 0), # atas
            (-1, 0), # bawah
            (1, 1), # kanan atas
            (1, -1), # kanan bawah
            (-1, 1), # kiri atas
            (-1, -1), # kiri bawah
            ]:
            if is_blocked(curr_node, dir, grid):
                continue

            neighbour = curr_node[0] + dir[0] , curr_node[1] + dir[1]
            if dir[0] != 0 and dir[1] != 0:
                tentative_gscore = gscore[curr_node] + 14
            else :
                tentative_gscore = gscore[curr_node] + 10
            
            if (neighbour in closed_list) :
                continue

            if tentative_gscore < gscore.get(neighbour, 0) or neighbour not in [i[1] for i in open_list]:
                came_from[neighbour] = curr_node
                gscore[neighbour] = tentative_gscore
                fscore[neighbour] = tentative_gscore
                heapq.heappush(open_list, (fscore[neighbour],neighbour))
            

        


    