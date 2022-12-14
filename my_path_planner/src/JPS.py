#!/usr/bin/env python

import heapq
import math
import time



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

def is_blocked(cur_x, cur_y, dir_x, dir_y, grid):
    if cur_x + dir_x < 0 or cur_x + dir_x >= len(grid):
        return True
    if cur_y + dir_y < 0 or cur_y + dir_y >= len(grid[1]):
        return True
    if dir_x != 0 and dir_y != 0 :
        if grid[cur_x + dir_x][cur_y] == obstacle and grid[cur_x][cur_y + dir_y] == obstacle :
            return True
        if grid[cur_x + dir_x][cur_y + dir_y] == obstacle :
            return True
    else:
        if dir_x != 0:
            if grid[cur_x + dir_x][cur_y] == obstacle:
                return True
        else :
            if grid[cur_x][cur_y + dir_y] == obstacle:
                return True
    return False

def hCost(curr_node, goal):
    xdist = math.fabs(goal[0] - curr_node[0])
    ydist = math.fabs(goal[1] - curr_node[1])

    if xdist > ydist:
        return 14 * ydist + 10 * (xdist - ydist)
    else :
        return 14 * xdist + 10 * (ydist - xdist)

def get_successors(cur_x, cur_y, came_from, grid, goal):
    successors = []
    neighbours = get_neighbours(cur_x, cur_y, came_from.get((cur_x, cur_y) ,0), grid)

    for node in neighbours:
        dX = node[0] - cur_x
        dY = node[1] - cur_y
        
        jump_point = jump(cur_x, cur_y, dX, dY, grid, goal)

        if jump_point != None:
            successors.append(jump_point)
    #print(successors)

    return successors

def direction (cur_x, cur_y, pX, pY):
    dX = int(math.copysign(1, cur_x- pX))
    dY = int(math.copysign(1, cur_y - pY))

    if cur_x - pX == 0:
        dX = 0
    if cur_y - pY == 0:
        dY = 0
    
    return (dX,dY)

def get_neighbours(cur_x, cur_y, parent, grid):
    neighbours = []
    if type(parent) != tuple:
        for i, j in [ (0, 1), # kanan
            (0, -1), # kiri
            (1, 0), # atas
            (-1, 0), # bawah
            (1, 1), # kanan atas
            (1, -1), # kanan bawah
            (-1, 1), # kiri atas
            (-1, -1) # kiri bawah
        ]:
            if not is_blocked(cur_x, cur_y, i, j, grid ):
                neighbours.append((cur_x + i, cur_y + j))
        
        return neighbours
    
    dX , dY = direction(cur_x, cur_y, parent[0], parent[1])

    if dX != 0 and dY != 0 :
        if not is_blocked(cur_x, cur_y, 0, dY, grid):
            neighbours.append((cur_x, cur_y + dY))

        if not is_blocked(cur_x, cur_y, dX, 0, grid):
            neighbours.append((cur_x + dX, cur_y))

        if (not is_blocked(cur_x, cur_y, 0, dY, grid) 
            or not is_blocked(cur_x, cur_y,dX, 0, grid)
            ) and not is_blocked(cur_x, cur_y, dX, dY, grid ):
            neighbours.append((cur_x + dX, cur_y + dY))
        
        if is_blocked(cur_x, cur_y, -dX, 0, grid) and not is_blocked(cur_x, cur_y, 0, dY, grid):
            neighbours.append((cur_x - dX, cur_y + dY))
        
        if is_blocked(cur_x, cur_y, 0, -dY, grid) and not is_blocked(cur_x, cur_y, dX, 0, grid):
            neighbours.append((cur_x + dX, cur_y - dY))
    
    else:
        if dX == 0:
            if not is_blocked(cur_x, cur_y, dX, 0, grid):
                if not is_blocked(cur_x, cur_y, 0, dY, grid):
                    neighbours.append((cur_x, cur_y + dY))
                if is_blocked(cur_x, cur_y, 1, 0, grid):
                    neighbours.append((cur_x + 1, cur_y + dY))
                if is_blocked(cur_x, cur_y, -1, 0, grid):
                    neighbours.append((cur_x - 1, cur_y + dY))
            
        else:
            if not is_blocked(cur_x, cur_y, dX, 0, grid):
                if not is_blocked(cur_x, cur_y, dX, 0, grid):
                    neighbours.append((cur_x + dX, cur_y))
                if is_blocked(cur_x, cur_y, 0, 1, grid):
                    neighbours.append((cur_x + dX, cur_y + 1))
                if is_blocked(cur_x, cur_y, 0, -1, grid):
                    neighbours.append((cur_x + dX, cur_y - 1))

    return neighbours

def dir_blocked(cur_x, cur_y, dir_x, dir_y, grid):
    if grid[cur_x - dir_x][cur_y] == obstacle and grid[cur_x][cur_y - dir_y]:
        return True
    else:
        return False

def jump(cur_x, cur_y, dir_x, dir_y, grid, goal):
  
    nX = cur_x + dir_x
    nY = cur_y + dir_y

    if is_blocked(nX, nY, 0, 0, grid):
        return None
    if (nX, nY) == goal:
        return (nX,nY)
    
    oX = nX
    oY = nY

    if dir_x != 0 and dir_y != 0:
        while True:
            if (
                not is_blocked(oX, oY, -dir_x, dir_y, grid)
                and is_blocked(oX, oY, -dir_x, 0, grid)
                or not is_blocked(oX, oY, dir_x, -dir_y, grid)
                and is_blocked(oX, oY, 0, -dir_y, grid)
            ):
                return (oX, oY)
            
            if (
                jump(oX, oY, dir_x, 0, grid, goal) != None
                or jump(oX, oY, 0, dir_y, grid, goal) != None
            ):
                return (oX, oY)
            
            oX += dir_x
            oY += dir_y

            if is_blocked(oX, oY, 0, 0, grid):
                return None

            if dir_blocked(oX, oY, dir_x, dir_y, grid):
                return None
            if (oX,oY) == goal:
                return (oX,oY)

    else:
        if dir_x != 0:
            while True:
                if (
                    not is_blocked(oX, nY, dir_x, 1, grid)
                    and is_blocked(oX, nY, 0, 1, grid)
                    or not is_blocked(oX, nY, dir_x, -1, grid)
                    and is_blocked(oX, nY, 0, -1, grid)
                ):
                    return(oX, nY)
                oX += dir_x

                if is_blocked(oX, nY, 0, 0, grid):
                    return None

                if (oX, nY) == goal:
                    return (oX, nY)
        
        else:
            while True:
                if (
                    not is_blocked(nX, oY, 1, dir_y, grid)
                    and is_blocked(nX, oY, 1, 0, grid)
                    or not is_blocked(nX, oY, -1, dir_y, grid)
                    and is_blocked(nX, oY, -1, 0, grid)
                ):
                    return(nX, oY)
                
                oY += dir_y

                if is_blocked(nX, oY, 0, 0, grid):
                    return None

                if (nX,oY) == goal:
                    return (nX, oY)
    
    return jump(nX, nY, dX, dY, grid, goal)

def lenght(curr_node, jump_point):
    dX , dY = direction( curr_node[0],curr_node[1], jump_point[0], jump_point[1])
    dX = math.fabs(dX)
    dY = math.fabs(dY)
    lX = math.fabs(curr_node[0] - jump_point[0])
    lY = math.fabs(curr_node[1] - jump_point[1])

    if dX != 0 and dY != 0:
        lenght = lX * 14
        return lenght
    else :
        lenght = (dX * lX + dY * lY) * 10
        return lenght

def search_jps(grid, start, goal):
    closed_list = set()
    came_from = {}
    gscore = {start : 0}
    fscore = {start : hCost(start, goal)}

    starttime = time.time()
    open_list = []
    heapq.heappush(open_list, (fscore[start], start))

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

        successors = get_successors(curr_node[0], curr_node[1], came_from, grid, goal)

        for successor in successors:
            jumpPoint = successor

            if jumpPoint in closed_list:
                continue
            
            tentative_gscore = gscore[curr_node] + lenght(curr_node, jumpPoint)

            if tentative_gscore < gscore.get(jumpPoint,0) or jumpPoint not in [j[1] for j in open_list]:
                came_from[jumpPoint] = curr_node
                gscore[jumpPoint] = tentative_gscore
                fscore[jumpPoint] = tentative_gscore + hCost(jumpPoint, goal)

                heapq.heappush(open_list, (fscore[jumpPoint], jumpPoint))




        