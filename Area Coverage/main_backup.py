import numpy as np
from generate_map import get_map
import matplotlib.pyplot as plt
from bresenham import bresenham
import math

class Robot():
    def __init__(self):
        self.side = 7
        self.all_points = []

    def paint_it(self, pos, map):
        color = np.linspace(0.2,0.8,len(pos))
        n = 0
        for p in pos:
            map[int(p[0]-(self.side - 1)/2):int(p[0] + (self.side + 1)/2) , int(p[1]-(self.side - 1)/2):int(p[1] + (self.side + 1)/2)] = color[n]
            n+=1

        return map

    def getpointsfromwps(self, wps):

        for i in range(len(wps)-1):
            pts = list(bresenham(wps[i][0],wps[i][1],wps[i+1][0],wps[i+1][1]))
            for j in range(len(pts)-1):
                self.all_points.append(pts[j])
        self.all_points.append(wps[-1])

def coverage_algo(start_pos, map, robot, thresh):
    wps = []
    wps.append(start_pos[0:2])
    
    # move to back edge (x,0)
    cr_pt = [thresh, start_pos[1]]
    wps.append(cr_pt)

    # move to the right corner (0,0)
    cr_pt = [thresh,thresh]
    wps.append(cr_pt)

    robot.getpointsfromwps(wps)
    flag = False
    
    while(not flag):
        while((map[cr_pt[0]+1,cr_pt[1]]== 0)|(map[cr_pt[0]+thresh,cr_pt[1]]== 0)):
            cr_pt = [cr_pt[0]+1,cr_pt[1]]
            wps.append(cr_pt)
            robot.all_points.append(cr_pt)
        
        for i in range(robot.side):
            if(map[cr_pt[0],cr_pt[1]+1]== 0):
                cr_pt = [cr_pt[0],cr_pt[1]+1]
                wps.append(cr_pt)
                robot.all_points.append(cr_pt)
            else:
                flag = True

        while((map[cr_pt[0]-1,cr_pt[1]] == 0)):
        
            cr_pt = [cr_pt[0]-1,cr_pt[1]]
            wps.append(cr_pt)
            robot.all_points.append(cr_pt)
    
        for i in range(robot.side):
            if(map[cr_pt[0],cr_pt[1]+1]== 0):
                cr_pt = [cr_pt[0],cr_pt[1]+1]
                wps.append(cr_pt)
                robot.all_points.append(cr_pt)
            else:
                flag = True
            
    return wps

def get_final_waypoints(init_waypoints, mp):
    final_waypoints = [init_waypoints[-1]]
    start_point = init_waypoints[-1]
    
    while(start_point is not init_waypoints[0]):

        found = 0
        idx = 0
        while(not found):
            end_point = init_waypoints[idx]
            points = list(bresenham(int(start_point[0]),int(start_point[1]),int(end_point[0]),int(end_point[1])))
            boom = 0
            
            for point in points:
                if mp[point[0],point[1]] != 0:
                    boom = 1
            
            if not boom:
                final_waypoints.append(end_point)
                found = 1
                start_point = end_point
            else:
                idx +=1

    return final_waypoints

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

#   final_waypoints = get_final_waypoints(initial_waypoints, mp)

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

if __name__ == "__main__":

    ## INPUTS:
    #          map, starting position

    map = get_map()
    robot = Robot()
    start = [30,30,0]
    thresh = int((robot.side+1)/2)

    waypoints = coverage_algo(start ,map, robot, thresh)
    
    
    robot.paint_it(robot.all_points, map)
    plt.imshow(map, cmap = 'gray_r')
    x_plot = []
    y_plot = []

    for p in robot.all_points:
        x_plot.append(p[0])
        y_plot.append(p[1])
    final = get_waypoints(waypoints, map)
    print(final)

    plt.plot(y_plot, x_plot)
    plt.show()