#!/usr/bin/env python

from JPS import *
from Astar import *
from Djikstra import *
import time
import Grid30x30
import Grid40x40
import Grid50x50
import Grid25x25
from visual import *

grid = [                       
[0, 0, 0, 0, 0], 
[0, 0, 1, 0, 0],
[0, 1, 1, 0, 0],
[0, 0, 0, 0, 0], 
[0, 0, 0, 0, 0],]

merah = (255, 0, 0)
hijau = (0, 255, 0)

def main():
    #grid = Grid50x50.maze
    start =tuple([0,0])
    goal = tuple([4,4])
    #path_Astar = search(grid, start, goal)
    (jps_path, jps_time) = search_jps(grid, start, goal)
    (astar_path, astar_time) = search(grid, start, goal)
    (djikstra_path, djikstra_time) = djikstra(grid, start, goal)
    # draw_grid(grid)
    print("################### A-Star #########################")
    print("path Astar = {}".format(astar_path))
    print("waktu Astar = {}".format(astar_time))
    print("panjang path = {}".format(path_lenght(astar_path)))
    #visualize(grid, path_Astar[0], hijau)

    print("################### Djikstra #########################")
    print("path Djikstra = {}".format(djikstra_path))
    print("waktu Djikstra = {}".format(djikstra_time))
    print("panjang path = {}".format(path_lenght(djikstra_path)))
    #visualize(grid, path_Astar[0], hijau)

    print("################### JPS #########################")
    #print(jps.method(grid,start, goal, 0))
    print("path JPS = {}".format(jps_path))
    print("waktu JPS = {}".format(jps_time))
    print("panjang path = {}".format(path_lenght(jps_path)))


    visualize(grid, astar_path)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        exit()
    