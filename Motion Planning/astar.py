import numpy as np
import math
import matplotlib.pyplot as plt
plt.ion()
import time
from bresenham import bresenham
from scipy.spatial import Voronoi

from robotplanner import robotplanner
from generate_map import get_map


# functions to time how long planning takes  
def tic():
  return time.time()
def toc(tstart, nm=""):
  return time.time() - tstart

# For Plotting Paths Only..
# Keep this commented while looking at the quantitative results like time taken.

def img(envmap, robotpos, targetpos, PHOTO1, PHOTO2, robot_list, target_list):

  color_map = plt.cm.get_cmap('gray')
  reversed_color_map = color_map.reversed()

  f, ax = plt.subplots()
  ax.imshow( envmap.T, interpolation="none", cmap=reversed_color_map, origin='lower',extent=(-0.5, envmap.shape[0]-0.5, -0.5, envmap.shape[1]-0.5), vmin = 0, vmax = 1 )
  ax.axis([-0.5, envmap.shape[0]-0.5, -0.5, envmap.shape[1]-0.5])
  ax.set_xlabel('x')
  ax.set_ylabel('y')
  hr = ax.plot(robotpos[0], robotpos[1], 'bs')
  ht = ax.plot(targetpos[0], targetpos[1], 'rs')

  robot_arr = get_arr(robot_list)
  target_arr = get_arr(target_list)

  plt.plot(robot_arr[:,0],robot_arr[:,1])
  plt.plot(target_arr[:,0],target_arr[:,1])

  f.canvas.flush_events()
  plt.show(block = True)


def get_arr(list_r):
  arr_r = np.zeros((len(list_r),2))
  for i in range(len(list_r)):
    arr_r[i] = np.array(list_r[i])
  return arr_r

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

  final_waypoints = get_final_waypoints(initial_waypoints, mp)

  return np.array(final_waypoints)

def voronoi_planner(map):

  points = []
  ct = 0
  for i in range(map.shape[0]):
    for j in range(map.shape[1]):
      ct+=1
      if(map[i,j] != 0):
        points.append([i,j])
        
  voronoi = Voronoi(points)
  vertices = voronoi.vertices

  vertices = [[int(v) for v in l] for l in vertices]
  new_v = []

  for vertex in vertices:
    if(map[vertex[0],vertex[1]] == 0):
      new_v.append(vertex)
  vertices = new_v

  # Create map for A star for vertices

  updated_map = map

  for i in range(map.shape[0]):
    for j in range(map.shape[1]):
      if(map[i,j] == 0):
        if([i,j] not in vertices):
          updated_map[i,j] = 0.3

  return updated_map

def test_map(map, curr_mode):
  robotstart = np.array([20, 20])
  targetpos = np.array([80, 80])
  
  map_used = 0
  waypoints = []

  if(curr_mode == mode['min_distance']):
    path = robotplanner(map, robotstart, targetpos, map_used)
    waypoints = get_waypoints(path, map)
    img(map, robotstart, targetpos, 0, 0, path, waypoints)
  
  if(curr_mode == mode['max_safety']):
    new_map = voronoi_planner(map)
    path = robotplanner(new_map, robotstart, targetpos, map_used)
    waypoints = get_waypoints(path, map)
    img(new_map, robotstart, targetpos, 0, 0, path, waypoints)

  return waypoints

mode = {
        "max_safety"   : 0,
        "min_distance" : 1
        }

def add_dimension(wp2):
  wp = []
  
  wp.append([wp2[0][0],wp2[0][1],0])
  
  for i in range(0,len(wp2)-1):
    current_wp = wp2[i]
    next_wp    = wp2[i+1]
    vector     = []
    vector.append(next_wp[0]-current_wp[0])
    vector.append(next_wp[1]-current_wp[1])
    l2_dist = math.sqrt(vector[0]*vector[0]+vector[1]*vector[1])
    vector_angle = math.acos(vector[0]/l2_dist)
    
    wp.append([current_wp[0],current_wp[1],vector_angle])
    wp.append([next_wp[0],next_wp[1],vector_angle])

  return wp

if __name__ == "__main__":
  
  map = get_map()
  waypoints_2 = test_map(map, mode['max_safety'])
  waypoints_3 = add_dimension(waypoints_2)
  print(waypoints_3)

  plt.ioff()
  # plt.show()