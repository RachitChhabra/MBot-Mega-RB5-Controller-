from mimetypes import init
import numpy as np
import math
from numpy import loadtxt
import matplotlib.pyplot as plt
import cv2
plt.ion()
import time
from bresenham import bresenham
from scipy.spatial import Voronoi
# from tqdm import tqdm

from robotplanner import robotplanner
from generate_map import get_map

# hyperparameters used for large maps
switch_length = 110      # Maximum path length for switching to normal A* again
seq_used = 100          # No. of times, previous A* result is used
scale_percent = 20      # Percent of original size

# functions to time how long planning takes  
def tic():
  return time.time()
def toc(tstart, nm=""):
  # print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))
  return time.time() - tstart

# For Plotting Paths Only..
# Keep this commented while looking at the quantitative results like time taken.

def img(envmap, robotpos, targetpos, PHOTO1, PHOTO2, robot_list, target_list):
  # for i in range(len(PHOTO1)):
  #   envmap[PHOTO1[i][0],PHOTO1[i][1]] = 0.2
  # for i in range(len(PHOTO1)):
  #   if((PHOTO1[i][0]<4900) & (PHOTO1[i][1]<4900)):
  #     for j in range(5):
  #       for k in range(5):
  #         envmap[(PHOTO1[i][0])*5-2+j,(PHOTO1[i][1])*5-2+k] = 0.2
  # envmap *= 255
  # color_map = plt.cm.get_cmap('cubehelix')
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

# For Plotting Paths Only..
# Keep this commented while looking at the quantitative results like time taken.

def get_arr(list_r):
  arr_r = np.zeros((len(list_r),2))
  for i in range(len(list_r)):
    arr_r[i] = np.array(list_r[i])
  return arr_r

def runtest(mapfile, robotstart, targetstart):
  t = tic()
  cost = 0
  # current positions of the target and robot
  robotpos = np.copy(robotstart);
  targetpos = np.copy(targetstart);
  large_map = False
  # environment
  envmap = loadtxt(mapfile)
  orig_map = envmap.copy()

  initial_dist = np.sqrt(np.square(robotpos[0] - targetpos[0]) + np.square(robotpos[1] - targetpos[1]))

  # Reducing the size of the map for larger maps.
  if (envmap.shape[0] > 4000):
    width = int(envmap.shape[1] * scale_percent / 100)
    height = int(envmap.shape[0] * scale_percent / 100)
    dim = (width, height)
    # resize image
    envmap = cv2.resize(envmap, dim)
    large_map = True
    robotpos[0] = int(robotpos[0]*(scale_percent/100))
    robotpos[1] = int(robotpos[1]*(scale_percent/100))
    targetpos[0] = int(targetpos[0]*(scale_percent/100))
    targetpos[1] = int(targetpos[1]*(scale_percent/100))
    print('map convoluted')
  # This does not remove any of the obstacles/walls.

  # now comes the main loop
  numofmoves = 0
  caught = False
  iter = 1
  started = 0
  path = []
  ROBOT = []
  TARGET = []
  map_used = 0
  flag = 0
  for i in range(20000):
    # call robot planner
    t0 = tic()

    # For plotting paths only.
    if (i == 0):
      ROBOT.append([(robotpos[0]),(robotpos[1])])
      TARGET.append([(targetpos[0]),(targetpos[1])])

    # Different conditions if the maps size is greater than a threshold.

    # if ((envmap.shape[0] > 999) & (initial_dist > 1500) & (len(path) > switch_length)):
    #   if (i == 0):
    #     newrobotpos, path = robotplanner(envmap, robotpos, targetpos, map_used)
    #   else:
    #     pass

      # if ((map_used == 0) & (iter < seq_used)):
      #   print('iter',iter)
      #   newrobotpos = path[-iter]
      #   iter+=1
      # else:
      #   newrobotpos, path = robotplanner(envmap, robotpos, targetpos, map_used)
      #   iter=1
    else:
      newrobotpos, path = robotplanner(envmap, robotpos, targetpos, map_used)
  
    # Total cost of the path.
    cost += np.sqrt(np.square(robotpos[0] - newrobotpos[0]) + np.square(robotpos[1] - newrobotpos[1]))

    #check that the new commanded position is valid
    if ( newrobotpos[0] < 0 or newrobotpos[0] >= envmap.shape[0] or \
         newrobotpos[1] < 0 or newrobotpos[1] >= envmap.shape[1] ):
      print('ERROR: out-of-map robot position commanded\n')
      break
    elif ( envmap[newrobotpos[0], newrobotpos[1]] != 0 ):
      print('ERROR: invalid robot position commanded\n')
      break
    elif (abs(newrobotpos[0]-robotpos[0]) > 1 or abs(newrobotpos[1]-robotpos[1]) > 1):
      print('ERROR: invalid robot move commanded\n')
      break

    # make the moves
    robotpos = newrobotpos

    # For plotting path only
    if(map_used == 0):
      ROBOT.append([(robotpos[0]),(robotpos[1])])
      TARGET.append([(targetpos[0]),(targetpos[1])])
    else:
      ROBOT.append([robotpos[0],robotpos[1]])
      TARGET.append([targetpos[0],targetpos[1]])
    numofmoves += 1
    
    if (abs(robotpos[0]-targetpos[0]) <= 1 and abs(robotpos[1]-targetpos[1]) <= 1):
      print('robotpos = (%d,%d)' %(robotpos[0],robotpos[1]))
      print('targetpos = (%d,%d)' %(targetpos[0],targetpos[1]))
      caught = True
      break
  
  # For data collection for results only
  totaltime = toc(t)

  return caught, numofmoves, cost, totaltime

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
  for i in range(map.shape[0]):
    for j in range(map.shape[1]):
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


def test_map(map, curr_mode, robotstart, targetpos):
  # robotstart = c
  # targetpos = np.array([90, 90])
  
  map_used = 0
  waypoints = []

  if(curr_mode == mode['min_distance']):
    path = robotplanner(map, robotstart, targetpos, map_used)
    waypoints = get_waypoints(path, map)
    # img(map, robotstart, targetpos, 0, 0, path, waypoints)
  
  if(curr_mode == mode['max_safety']):
    new_map = voronoi_planner(map)
    path = robotplanner(new_map, robotstart, targetpos, map_used)
    waypoints = get_waypoints(path, map)
    # img(new_map, robotstart, targetpos, 0, 0, path, waypoints)

  return waypoints

# tags = {
#         1:[],
#         2:[],
#         3:[],
#         4:[],
#         5:[],
#         6:[],
#         7:[],
#         8:[]
#         }

mode = {
        "max_safety"   : 0,
        "min_distance" : 1
        }

def plan_path(robotstart, targetpos, m):
  map = get_map()
  
  waypoints = test_map(map, mode[m], robotstart, targetpos)
  return waypoints

if __name__ == "__main__":
  # you should change the following line to test different maps

  # map = np.loadtxt('map100m.txt')
  map = get_map()
  waypoints = test_map(map, mode['max_safety'])

  plt.ioff()
  # plt.show()