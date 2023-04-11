import numpy as np
from generate_map import get_map
import matplotlib.pyplot as plt
from bresenham import bresenham
import math
from robotplanner import robotplanner
import matplotlib.path as mplPath

class Robot():
    def __init__(self):
        self.side = 7
        self.all_points = []

    def paint_it1(self, pos, map):
        color = np.linspace(0.2,0.8,len(pos))
        n = 0
        for p in pos:
            map[int(p[0]-(self.side - 1)/2):int(p[0] + (self.side + 1)/2) , int(p[1]-(self.side - 1)/2):int(p[1] + (self.side + 1)/2)] = color[n]
            n+=1

        return map

    def paint_it2(self, pos, map):
        color = np.linspace(0.2,0.8,len(pos))
        n = 0
        for p in pos:
            map[int(p[0]-(self.side - 1)/2):int(p[0] + (self.side + 1)/2) , int(p[1]-(self.side - 1)/2):int(p[1] + (self.side + 1)/2)] = 1
            n+=1

        return map

    def getpointsfromwps(self, wps):

        for i in range(len(wps)-1):
            pts = list(bresenham(wps[i][0],wps[i][1],wps[i+1][0],wps[i+1][1]))
            for j in range(len(pts)-1):
                self.all_points.append(pts[j])
        self.all_points.append(wps[-1])

def coverage_algo(start_pos, map, robot, thresh, orig_map):

    corners = [0]
    iter = 0
    wps = []
    map = minkowski(map,robot)

    while(1):
        print(iter)
        corners = np.array(get_corners(map))

        if(len(corners)==0):
            break

        if(len(corners)>3):
            corners[[-2, -1]] = corners[[-1, -2]]

        corners = list(corners)

        for i in range(len(corners)-1):
            path = robotplanner(map, corners[i], corners[i+1], None)
            wps = add_wps_to_path(wps, path)
        path = robotplanner(map, corners[-1], corners[0], None)
        wps = add_wps_to_path(wps, path)
        map = robot.paint_it2(wps, map)
        start_pos = corners[0]
        iter+=1

    robot.getpointsfromwps(wps)
    return wps, orig_map, start_pos

def get_dist(x1, x2):
    x1 = np.array(x1[0:2])
    x2 = np.array(x2[0:2])
    return np.sqrt(np.sum((x1 - x2)**2))

def add_wps_to_path(wps, path):
    for p in path:
        wps.append(p)
    return wps

def get_corners(map):
    corners = []
    for i in range(1,map.shape[0]-1):
        for j in range(1,map.shape[1]-1):
            black = 0
            for ix in [-1,0,1]:
                for iy in [-1,0,1]:
                    if(map[i+ix,j+iy] == 1):
                        black +=1
            if ((black > 3)&(black < 6)):
                corners.append([i,j])
    return corners

def minkowski(map, robot):
    robot_width = robot.side 

    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            if(map[i,j] == 1):
                for k in range(max(i-int(robot_width/2),0), min(i+int(robot_width/2)+1,map.shape[0])):
                    for l in range(max(j-int(robot_width/2),0), min(j+int(robot_width/2)+1,map.shape[1])):
                        map[k,l] = max(map[k,l],0.99)

    map[map == 0.99] = 1
    return map


def get_points_angle_change(path):
    ret = []
    th = 0.0
    th_prev = 0.0
    x_prev = None
    y_prev = None
    for x,y in path:
        X = x
        Y = y
        if x_prev is not None:
            if Y-y_prev and X-x_prev:
                temp = np.pi/4
            elif Y-y_prev:
                temp = np.pi/2
            elif X-x_prev:
                temp = 0
            else:
                raise NotImplementedError
            
            if temp != th_prev:
                th = temp
                th_prev = th        
                ret.append((x_prev,y_prev)) 
        x_prev = X
        y_prev = Y
    ret.append((path[-1][0],path[-1][1]))
    return ret

def get_waypoints(path, mp):

  initial_waypoints = get_points_angle_change(path)
  final_waypoints = add_dimension(initial_waypoints)

  return np.array(final_waypoints)

def add_dimension(wp2):
  wp = []
  ## Turn -> 1
  ## Straight -> 0
  lc = 0.0245
  wp.append([wp2[0][0]*lc,wp2[0][1]*lc,0,0])
  
  for i in range(0,len(wp2)-1):
    current_wp = wp2[i]
    next_wp    = wp2[i+1]
    vector     = []
    vector.append(next_wp[0]-current_wp[0])
    vector.append(next_wp[1]-current_wp[1])
    l2_dist = math.sqrt(vector[0]*vector[0]+vector[1]*vector[1])
    vector_angle = math.acos(abs(vector[0])/l2_dist)

    wp.append([current_wp[0]*lc,current_wp[1]*lc,vector_angle,1])
    wp.append([next_wp[0]*lc,next_wp[1]*lc,vector_angle,0])

  return wp



def get_waypoints_for_coverage():

    ## INPUTS:
    #          map, starting position

    map = get_map()
    
    robot = Robot()
    start = [50,20,0]
    thresh = int((robot.side+1)/2)
    
    corners = np.array(get_corners(map))

    boundary_corners = []
    obstacle_corners = []
    
    ## Find Corners for the given map
    for corner in corners:
        if(map[corner[0],corner[1]] == 1):
            obstacle_corners.append(corner)
        else:
            boundary_corners.append(corner)

    waypoints_overall = []
    if(len(obstacle_corners)>0):
        boundary_corners = np.array(boundary_corners)
        boundary_corners[[-1,-2]] = boundary_corners[[-2,-1]]
        boundary_corners = list(boundary_corners)

        obstacle_corners = np.array(obstacle_corners)
        obstacle_corners[[-1,-2]] = obstacle_corners[[-2,-1]]
        obstacle_corners = list(obstacle_corners)

        boundary_corners.append(boundary_corners[0])
        obstacle_corners.append(obstacle_corners[0])

        all_maps = {}  
        iter = 0
    
        for c in range(4):
            temp_map = np.zeros(map.shape)
            for i in range(map.shape[0]):
                for j in range(map.shape[1]):
                    poly_path = mplPath.Path(np.array([[boundary_corners[c][0], boundary_corners[c][1]],
                                            [boundary_corners[c+1][0], boundary_corners[c+1][1]],
                                            [obstacle_corners[c+1][0], obstacle_corners[c+1][1]],
                                            [obstacle_corners[c][0], obstacle_corners[c][1]]]))
                    point = (i,j)
                    if poly_path.contains_point(point):
                        temp_map[i,j] = 0
                    else:
                        temp_map[i,j] = 1

            all_maps[iter] = temp_map
            iter+=1

        
        for i in range(0,len(boundary_corners)-1):

            waypoints, map, start = coverage_algo(start ,all_maps[i], robot, i, map)
            for wp in waypoints:
                waypoints_overall.append(wp)

    waypoints, map, start = coverage_algo(start ,map, robot, 0, map)
    for wp in waypoints:
        waypoints_overall.append(wp)


    map = robot.paint_it1(waypoints_overall, map)
    plt.imshow(map, cmap = 'gray_r')
    x_plot = []
    y_plot = []

    for p in waypoints_overall:
        x_plot.append(p[0])
        y_plot.append(p[1])

    final = get_waypoints(waypoints, map)

    plt.plot(y_plot, x_plot, 'b--', markersize=2, markevery=2)
    plt.show()

