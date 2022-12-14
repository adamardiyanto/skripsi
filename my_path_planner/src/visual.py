#!/usr/bin/env python


import pygame


one = (255, 0, 0)
two = (206, 171, 147)

free = (255, 255, 255)
obstacle = (0, 0, 0)
merah = (255, 0, 0)
hijau = (0, 255, 0)
biru = (0, 0, 255)

merah_muda = (100, 0, 0)
hijau_muda = (0, 100, 0)
biru_muda = (0, 0, 100)

width = 10
height = 10
margin = 1


# grid = [                       
# [0, 0,  0,  0,  0], 
# [0, 0,  1,  0,  0], 
# [0, 1,  1,  0,  0], 
# [0, 0,  0,  0,  0], 
# [0, 0,  0,  0,  0] ]



def visualize(grid, path1, path2=None, path3=None):
    pygame.init()
    size_x = len(grid) * (width + margin + margin) 
    size_y = len(grid[1]) * (height + margin + margin)
    size = (size_y, size_x)
    screen = pygame.display.set_mode(size)

    pygame.display.set_caption("MAZE")


    while True:
        
        #print(grid)
        for node in path1:
            grid[node[0]][node[1]] = 2
        if path2 != None or path3 != None:
            
            for node in path2:
                grid[node[0]][node[1]] = 3

            for node in path3:
                grid[node[0]][node[1]] = 4


        screen.fill(two)
        for row in range(len(grid)):
            for column in range(len(grid[1])):
                if grid[row][column] == 0:
                    color = free
                elif grid[row][column] == 1:
                    color = obstacle
                elif grid[row][column] == 2:
                    color = merah_muda
                elif grid[row][column] == 3:
                    color = hijau_muda
                elif grid[row][column] == 4:
                    color = biru_muda
                pygame.draw.rect(screen, color, [margin + (margin + width) * column, margin + (margin + height) * row, width, height])
        
        if path2 == None or path3 == None:
            for wp in range(len(path1)-1):
                a = path1[wp]
                b = path1[wp + 1]

                pygame.draw.line(screen, biru, (a[1] * (margin + width) + (margin + width)/2, a[0] * (margin + height) + (margin + height)/2 ),
                                (b[1] * (margin + width) + (margin + width)/2, b[0] * (margin + height) + (margin + height)/2), width= 5)
        else:
            for wp in range(len(path1)-1):
                a = path1[wp]
                b = path1[wp + 1]

                pygame.draw.line(screen, merah, (a[1] * (margin + width) + (margin + width)/2, a[0] * (margin + height) + (margin + height)/2 ),
                                (b[1] * (margin + width) + (margin + width)/2, b[0] * (margin + height) + (margin + height)/2), width= 5)
            for wp in range(len(path2)-1):
                a = path2[wp]
                b = path2[wp + 1]

                pygame.draw.line(screen, hijau, (a[1] * (margin + width) + (margin + width)/2, a[0] * (margin + height) + (margin + height)/2 ),
                                (b[1] * (margin + width) + (margin + width)/2, b[0] * (margin + height) + (margin + height)/2), width= 5)
            for wp in range(len(path3)-1):
                a = path3[wp]
                b = path3[wp + 1]

                pygame.draw.line(screen, biru, (a[1] * (margin + width) + (margin + width)/2, a[0] * (margin + height) + (margin + height)/2 ),
                                            (b[1] * (margin + width) + (margin + width)/2, b[0] * (margin + height) + (margin + height)/2), width= 5)

        astar = pygame.font.Font.render(pygame.font.SysFont("bahnschrift", 30), "Astar", True, merah)
        djkstra = pygame.font.Font.render(pygame.font.SysFont("bahnschrift", 30), "Djikstra", True, hijau)
        jps = pygame.font.Font.render(pygame.font.SysFont("bahnschrift", 30), "JPS", True, biru)

        screen.blit(astar, (160, size_x - 30))
        screen.blit(djkstra, (70, size_x - 30))
        screen.blit(jps, (20, size_x - 30))
        pygame.display.flip()

        
        
    