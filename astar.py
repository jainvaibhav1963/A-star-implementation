# -*- coding: utf-8 -*-
"""
Created on Sun Apr 11 14:10:06 2021

@author: jain
"""

import numpy as np
import cv2 
import matplotlib.pyplot as plt
import math


###########################################################################################################33
# define maze
maze = np.zeros((300,400))

for i in range(300):
    
    for j in range(400):
    
        # three rectangles
        if (i >= 20) and (i <=70) and (j >= 200) and (j <= 210):
            maze[i][j] = 1
        if (i >= 60) and (i <=70) and (j >= 210) and (j <= 230):
            maze[i][j] = 1
        if (i >= 20) and (i <=30) and (j >= 210) and (j <= 230):
            maze[i][j] = 1 
        
            # circle
        if ((j-90)**2+(i-(-70+300))**2 <= 35**2):
            maze[i][j] = 1
         
            # Ellipse
        if ((j-246)/60)**2 + ((i-155)/30)**2 <= 1 :
             maze[i][j] = 1
            
            #  inclined rectangle
        if (-0.7*j+1*(300-i))>=73.4 and ((300-i)+1.428*j)>=172.55 and (-0.7*j+1*(300-i))<=99.81 and ((300-i)+1.428*j)<=429:
            maze[i][j] = 1
            
################################################################################################################3
radius = int(input('enter robot raius: '))
clear =int(input('input robot clearance: '))
##################################################################################3
# Check source node surroundings
# modify obstacles to account for clearance
# Identify all edge points of obstacles and then draw clearance circle treating each point as the center to a circle
maze1 = maze.copy()

[row,col] = np.where(maze1 == 1)
r = []
c = []
for b in range(len(row)):
    
      if maze1[row[b]][col[b]+1] != 1 or maze1[row[b]][col[b]-1] != 1 or maze1[row[b]+1][col[b]] != 1 or maze1[row[b]-1][col[b]] != 1:
          r.append(row[b])
          c.append(col[b])
    
for b in range(len(r)):
    for i in range(300):  
        for j in range(400):
                # circle
            if ((j-c[b])**2+(i-r[b])**2 <= ((clear+radius)/2)**2):
                maze1[i][j] = 1
    # plotting maze with clearance to see what it looks like
Modified_maze = maze1.copy()
plt.imshow(Modified_maze),plt.show()
######################################################################################################33

# make every zero to be infinity (9999)
[row,col] = np.where(maze1 == 0)
for b in range(len(row)):
    maze1[row[b]][col[b]] = 9999

#########################################3
###########################################
# start and goal points
# start = [0,0]
# goal = [50,300]
start1 = []  
  
# For list of strings/chars
goal1 = []  
  
start1 = [int(item) for item in input("Enter the start coordinates : ").split()]
  
goal1 = [int(item) for item in input("Enter the goal coorinates : ").split()]

start = [300 - start1[1], start1[0]]
goal = [300 - goal1[1], goal1[0]]



# step size
# d = 6
d = int(input('Enter the step size (enter 6 for default process):'))
# theta
th = (math.pi)/6

x = round(d*math.cos(th))
y = round(d*math.sin(th))



# threshhold, node size
# z = 2
z = float(input('Enter threshold for x,y (enter 2 for default process): '))
# rr = 300
# cc = 400

# if z < 1:
#     rr = (1/z)*300
#     cc = (1/z)*400
    
#     z = 1
############################################3
########################################33###

# queues
visited_nodes = []
open_nodes = []

# making start node to be zero
maze1[start[0]][start[1]] = 0
# get an circular area of goal nodes according to threshhold
trial = maze.copy()
goals = []
for i in range(300):        
    for j in range(400):
            # circle
        if ((j-goal[1])**2+(i-goal[0])**2 <= z**2):
            trial[i][j] = 2
[row,col] = np.where(trial == 2)
for b in range(len(row)):
    goals.append([row[b],col[b]])


open_nodes.append([start[0], start[1]])   # putting start position in the open node list

# maze copies
cost_to_come = maze1.copy()
cost_to_go = maze1.copy()
total_cost = maze1.copy()


# cost_to_go maze calculation
# goal in cartesian
g = np.array([goal[1], 300 - goal[0]])
for m in range (300):
    for n in range (400):
        #current node
        a = np.array([n, 300-m])
        # distance of current node from goal
        cost_to_go[m][n] = np.linalg.norm(a-g)

        

#########################################################################
# movement functions
def N(maze1):
        # make sure robot can move new position
        if (p-d >= 0) and maze1[p-d][q] > maze1[p][q] + d:  # check weight of new position, if cost to come is lower than previously estimated - change it
            maze1[p-d][q] = maze1[p][q] + d
            open_nodes.append([p-d,q])  # put node in open node list
    
            return maze1
        else:  # if cost to come for the new position was lower then dont do anything
            return maze1
    

def S(maze1):

       if (p+d <= 299) and maze1[p+d][q] > maze1[p][q] + d:
           maze1[p+d][q] = maze1[p][q] + d
           open_nodes.append([p+d,q])  # put node in open node list
                
           return maze1    
       else:
           return maze1
    

def E(maze1):
    
       if (q+d <= 399) and maze1[p][q+d] > maze1[p][q] + d:
           maze1[p][q+d] = maze1[p][q] + d
           open_nodes.append([p,q+d])  # put node in open node list  
        
           return maze1
       else:
           return maze1
    

def W(maze1):

       if (q-d >= 0) and maze1[p][q-d] > maze1[p][q] + d:
           maze1[p][q-d] = maze1[p][q] + d
           open_nodes.append([p,q-d])

           return maze1
       else:
           return maze1
       

# def NW(maze1):
    
#     if (maze1[p-x][q-y] > maze1[p][q] + 2**(.5)) and (p-x >= 0) and (q-y >= 0):
#         maze1[p-x][q-y] = maze1[p][q] + 2**(.5)
#         open_nodes.append([p-x,q-y])
        
#         return maze1
#     else:
#         return maze1
    
    
def NE(maze1):
    

    if (p-x >= 0) and (q+y <= 399) and (maze1[p-x][q+y] > maze1[p][q] + d**(.5)):
        maze1[p-x][q+y] = maze1[p][q] + d**(.5)
        open_nodes.append([p-x,q+y])
        
    if (p-x >= 0) and (q-y >= 0) and (maze1[p-x][q-y] > maze1[p][q] + d**(.5)):
        maze1[p-x][q-y] = maze1[p][q] + d**(.5)
        open_nodes.append([p-x,q-y])
        
    if (q+y <= 399) and (p+x <= 299) and (maze1[p+x][q+y] > maze1[p][q] + d**(.5)):
        maze1[p+x][q+y] = maze1[p][q] + d**(.5)
        open_nodes.append([p+x,q+y])
        
    if (q-y >= 0) and (p+x <= 299) and (maze1[p+x][q-y] > maze1[p][q] + d**(.5)):
        maze1[p+x][q-y] = maze1[p][q] + d**(.5)
        open_nodes.append([p+x,q-y])
        
        return maze1
    else:
        return maze1


# def SE(maze1):

#     if (p+d <= 299) and (q+d <= 399) and (maze1[p+d][q+d] > maze1[p][q] + 2**(.5)):
#         maze1[p+d][q+d] = maze1[p][q] + 2**(.5)
#         open_nodes.append([p+d,q+d])
        
#         return maze1
#     else: 
#         return maze1
    


# def SW(maze1):
    
#     if (p+d <= 299) and (maze1[p+d][q-d] > maze1[p][q] + 2**(.5)) and (q-d >= 0):
#         maze1[p+d][q-d] = maze1[p][q] + 2**(.5)
#         open_nodes.append([p+d,q-d])

#         return maze1  
#     else:
#         return maze1


#########################################################################333

# check to see if initial and goal nodes are in obstacle space

if maze1[goal[0]][goal[1]] == 1:
    print(' The goal is in obstacle space, Please provide another goal node')
    
elif maze1[start[0]][start[1]] == 1:
    print(' The start position is in obstacle space, Please provide another goal node')
    
else:
    
    vid = []       

# main
    while open_nodes != []:   # while open node list is not empty
    
      
  
        img = maze1.copy()
        vid.append(img)  # frames for video
    
        val_list = []
        for i in range(len(open_nodes)):
            [h,k] = open_nodes[i]
            val = total_cost[h][k]
            val_list.append(val)        
            
            min_index = val_list.index(min(val_list))
            
        [p,q] = open_nodes[min_index]   # move robot to new source
        
        # when is goal reched
        for element in open_nodes:
            if element in goals:
                print("goal_reached")
                goals.append([p,q])
        # [p,q] is in any node in a circle around goal node
        if [p,q] in goals:   # if robot is in goal node then stop the loop
            break
        
        if [p,q] not in visited_nodes:
            
            # do movements
            maze1 = N(maze1)
            maze1 = S(maze1)
            maze1 = E(maze1)
            maze1 = W(maze1)
            maze1 = NE(maze1)
            # maze1 = NW(maze1)
            # maze1 = SE(maze1)
            # maze1 = SW(maze1)
            
            # update cost_to_come maze after each iteration
            cost_to_come = maze1.copy()
            total_cost = cost_to_go + cost_to_come
            
            # set current node and all the nodes that belong in a circle with p,q as centre and z as radius to be in visited node list
            # make a matrix of 300x400 of all zeroes
            visited_matrix = np.zeros((300,400))
            # make the circle values to be 1
            for i in range(300):        
                for j in range(400):
                        # circle
                    if ((j-q)**2+(i-p)**2 <= z**2):
                        visited_matrix[i][j] = 1
            # append all indices that are 1 in the visited list
            [row,col] = np.where(visited_matrix == 1)
            for b in range(len(row)):
                visited_nodes.append([row[b],col[b]])
                    
            # to remove duplicated 
            # from list 
            res = []
            [res.append(x) for x in open_nodes if x not in res]
            open_nodes = res
            
            # take out nodes from open list that are in visited node list
            for element in visited_nodes:
                if element in open_nodes:
                    open_nodes.remove(element)
            
            # # arrnge open_nodes according to lowest total cost
            # val_list = []
            # for i in range(len(open_nodes)-1):
            #     [h,k] = open_nodes[i]
            #     val = total_cost[h][k]
            #     val_list.append(val)
            
            # trial_val_list = val_list    
            # val_list.sort()
            # open_nodes1 = open_nodes
            
            # for i in range(len(open_nodes)):
            #     open_nodes[i] = open_nodes1[trial_val_list.index(val_list[i])]
      
    ###########################################################################################################
    
    
    
    ###################################################################
    # optimal path
    print("optimal_path")
    maze2 = total_cost.copy()
    
    # convert all 1 to 9999
    [row,col] = np.where(maze1 == 1)
    for b in range(len(row)):
        maze2[row[b]][col[b]] = 9999
    
    path_nodes = []   # list that depict nodes for the optimal path
    
    #path_nodes.append(goal)   # add goal node to the list
    
    [m,n] = start   # indices
    
    while [m,n] != [p,q]:   # loop until path goes from goal to start node
        # choose the smallest neighbor as the next node
        path_nodes.append([m,n])         # add to path
        maze2[m][n] = 7777
        
        neighbor_values = [maze2[m+d][n], maze2[m-d][n], maze2[m][n+d], maze2[m][n-d],
                            maze2[m+x][n+y], maze2[m-x][n-y], maze2[m-x][n+y], maze2[m+x][n-y]]
        
        min_value = min(neighbor_values)
        min_value_indices = neighbor_values.index(min_value)   # indices for smallet neighbouring value
        
        if min_value_indices == 0:
            [m,n] = [m+d,n]
            continue
            
        elif min_value_indices == 1:
            [m,n] = [m-d,n]
            continue
            
        elif min_value_indices == 2:
            [m,n] = [m,n+d]
            continue
            
        elif min_value_indices == 3:
            [m,n] = [m,n-d]
            continue
            
        elif min_value_indices == 4:
            [m,n] = [m+x,n+y]
            continue
            
        elif min_value_indices == 5:
            [m,n] = [m-x,n-y]
            continue
            
        elif min_value_indices == 6:
            [m,n] = [m-x,n+y]
            continue
    
        elif min_value_indices == 7:
            [m,n] = [m+x,n-y]
            continue
            
        # path_nodes.append([m,n])         # add to path
        # maze2[m][n] = 7777

    # Trim number of images if necessary
    while len(vid) > 1200:
        vid = vid[::2]
        
       
    # make the images better
    for i in range(len(vid) - 1):
        
        edit =  vid[i]
        # change untravelled nodes to zero 
        [row,col] = np.where(edit == 9999)
        for b in range(len(row)):
            edit[row[b]][col[b]] = 0
        # change all path nodes to some shade
        [row,col] = np.where(edit > 1)
        for b in range(len(row)):
            edit[row[b]][col[b]] = 100
        #  chenge all obstacles to 255 in to get black in greyscale.
        [row,col] = np.where(edit == 1)
        for b in range(len(row)):
            edit[row[b]][col[b]] = 255
        
            # save the frame as png
        cv2.imwrite(str(i) + '.png', vid[i])
        
    
    
    # get an imaage for optimal path
    img1 = maze2.copy()
    
    #     # change untravelled nodes to zero 
    # [row,col] = np.where(img1 == 9999)
    # for b in range(len(row)):
    #     img1[row[b]][col[b]] = 0
        # change all path nodes to negative
    [row,col] = np.where(img1 == 7777)
    for b in range(len(row)):
        img1[row[b]][col[b]] = -1
        # change all visited nodes except path nodes
    [row,col] = np.where(img1 > 1)
    for b in range(len(row)):
        img1[row[b]][col[b]] = 0
      # give path nodes some intensity for image formation  
    [row,col] = np.where(img1 == -1)
    for b in range(len(row)):
        img1[row[b]][col[b]] = 175
        # make obstacles black 
    [row,col] = np.where(img1 == 1)
    for b in range(len(row)):
        edit[row[b]][col[b]] = 255
      # give start and goal node lighter intensity  
    img1[start[0]][start[1]] = 75
    img1[goal[0]][goal[1]] = 75
    
    # save image
    cv2.imwrite('optimal_path.png', img1)
    
      
        
    #################################################################   
    # video of path finding
    
    
    # gif to get an idea of what video will look like
    # # Create the frames
    # frames = []
    # imgs = glob.glob("*.png")
    # for i in imgs:
    #     new_frame = Image.open(i)
    #     frames.append(new_frame)
     
    # # Save into a GIF file that loops forever
    # frames[0].save('png_to_gif.gif', format='GIF',
    #                append_images=frames[1:],
    #                save_all=True,
    #                duration=3000, loop=0)
    
    # # save all frames as images
    # for i in range(len(vid) - 1):
        
    #     cv2.imwrite(str(i) + '.png', vid[i])
        
    
    img=[]
    for i in range(0,len(vid)):
        img.append(cv2.imread(str(i)+'.png'))
    
    height,width,layers=img[1].shape
    
    img1 = cv2.imread('optimal_path.png')
    img.append(img1)
    video=cv2.VideoWriter('video_A_star.mp4',-1,30,(width,height))
    
    
    
    for j in range(len(vid)+1):
    
        video.write(img[j])
    
    cv2.destroyAllWindows()
    video.release()
    ######################################################################################   
    
    
    # lets make another video for optimal path in original maze
    
    # variable maze is the original maze array
    
    [row,col] = np.where(maze == 1)
    for b in range(len(row)):
        maze[row[b]][col[b]] = 255
    
    frames = []
    
    while path_nodes != []:
        
        [r,c] = path_nodes.pop(0)
        maze[r][c] = 150
        
        frame = maze.copy()
        frames.append(frame)   
        #cv2.imwrite(str(i) + 'path.png', frame)
        
    for i in range(len(frames)-1):
        cv2.imwrite(str(i) + 'path.png', frames[i])
        
        
    
    # video maker
    imge=[]
    for i in range(0,len(frames)):
        imge.append(cv2.imread(str(i)+'path.png'))
    
    height,width,layers=imge[1].shape
    
    img1 = cv2.imread('optimal_path.png')
    #imge.append(img1)
    video=cv2.VideoWriter('video_optimal_path.mp4',-1,10,(width,height))
    
    
    
    for j in range(len(frames)):
    
        video.write(imge[j])
    
    cv2.destroyAllWindows()
    video.release()
        
 ######################################################################################   
        

    
    
    
    
    
        
    

    
    
        
    
    
    
    

    





