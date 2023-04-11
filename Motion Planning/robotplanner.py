import numpy as np
from pqdict import pqdict
import time
import json
import matplotlib.pyplot as plt
from tqdm import tqdm

# Hyperparameter
e = 0

# Initializing required variables
inf = 100000
GRAPH = {}

# For plotting results only
PHOTO1 = []
PHOTO2 = []

class Environment(object):
  def __init__ (self, envmap, length, width, goal_pos, start_pos):
    self.envmap = envmap
    self.length = length
    self.width = width
    self.goal = goal_pos
    self.start = start_pos
    self.children = []
  
  # Outputs successors for inputed node.
  def getSuccessors(self, node):
    # Initializing empty lists for storing Successors and Cost
    Successors = []
    cij = []

    numofrobodirs = 8
    robot_dX = [-1, -1, -1,  0,  1,  1,  1,  0]
    robot_dY = [-1,  0,  1,  1,  1,  0, -1, -1]

    for robo_dd in range(numofrobodirs):
        
        robot_newx = node[0] + (robot_dX[robo_dd])
        robot_newy = node[1] + (robot_dY[robo_dd])
        
        if (robot_newx >= 0 and robot_newx < self.length and robot_newy >= 0 and robot_newy < self.width):
          if(self.envmap[robot_newx, robot_newy] == 0):     # Not obstacles
            Successors.append([robot_newx, robot_newy])
          
            if robo_dd%2 == 1:
              cij.append(1)
            else:
              cij.append(1.414)
 
    return Successors, cij

  def getHeuristic(self, node):
    # Both euclidian and manhattan distance were tried for the Heuristic.
    # manhattan_dist = abs(node[0] - self.goal[0]) + abs(node[1] - self.goal[1])
    euclidian_dist = np.sqrt(np.square(node[0] - self.goal[0]) + np.square(node[1] - self.goal[1]))
    # return manhattan_dist
    return euclidian_dist
 
# NODE class for storing all the variables.
class NODE(object):
  def __init__ (self, xy):
    self.xy = xy
    self.g = inf
    self.h = 0
    self.f = self.g + e*self.h
    self.parent_id = np.array([])
    self.CLOSED = 0

  # to obtain F value at every iteration.
  def getF(self):
    self.f = self.g + e*self.h
    return (self.g+e*self.h)


def robotplanner(envmap, robotpos, targetpos, map_used):
  newrobotpos = np.copy(robotpos)
  newtargetpos = np.copy(targetpos)
  path = AstarFunction(envmap, newrobotpos, newtargetpos, map_used)
  newrobotpos[0] = int(path[-1][0])
  newrobotpos[1] = int(path[-1][1])
  newrobotpos = np.array([newrobotpos[0],newrobotpos[1]])
  GRAPH.clear()
  
  return path

# For plotting results only
def get_PHOTO():
  return PHOTO1, PHOTO2

def AstarFunction(envmap, robotpos, targetpos, map_used):

  # Initializing Environment and Node Objects
  env = Environment(envmap, envmap.shape[0], envmap.shape[1], targetpos, robotpos)
  start_node = NODE(robotpos)
  goal_node = NODE(targetpos)
  #  Initializing h values for the start
  start_node.h = env.getHeuristic(start_node.xy)

  #  Initializing g value for start node.
  start_node.g = 0
  GRAPH[str([robotpos[0],robotpos[1]])] = start_node
  GRAPH[str([targetpos[0],targetpos[1]])] = goal_node

  # create OPEN priority queue and add Start position
  OPEN = pqdict()
  OPEN.additem(str([robotpos[0],robotpos[1]]),start_node.getF())
  # For plotting results only
  PHOTO1.append([robotpos[0],robotpos[1]])
  n = 0
  while GRAPH[str([targetpos[0],targetpos[1]])].CLOSED == 0:
    n+=1
    # Remove the highest priority node from the OPEN list
    robotpos = json.loads(OPEN.pop())
    # Mark the removed node as CLOSED.
    GRAPH[str([robotpos[0],robotpos[1]])].CLOSED = 1
    robotpos = GRAPH[str([robotpos[0],robotpos[1]])].xy
    expand_node(env, robotpos, targetpos, OPEN, map_used)

  path = []
  path = recover_path(env, robotpos, targetpos)

  return path

# Function to Expand Nodes
def expand_node(env, robotpos, targetpos, OPEN, map_used):
  children, cost = env.getSuccessors(robotpos)
  child_no = 0
  for child in children:
    if (str(child) in GRAPH):
      if (GRAPH[str(child)].CLOSED == 0):
        if (GRAPH[str(child)].g > GRAPH[str([robotpos[0],robotpos[1]])].g + cost[child_no]):
          GRAPH[str(child)].g = GRAPH[str([robotpos[0],robotpos[1]])].g + cost[child_no]
          GRAPH[str(child)].parent_id = robotpos
    else:
      child_node = NODE(child)
      child_node.g = GRAPH[str([robotpos[0],robotpos[1]])].g + cost[child_no]
      child_node.parent_id = robotpos
      child_node.h = env.getHeuristic(child_node.xy)
      GRAPH[str(child)] = child_node
    if (GRAPH[str(child)].CLOSED == 0):
      if(str(child) in OPEN):
        OPEN.updateitem(str(child), GRAPH[str(child)].getF())
      else:
        OPEN.additem(str(child), GRAPH[str(child)].getF())
        # For plotting results only
        if (map_used == 0):
          PHOTO1.append([child[0],child[1]])
        if (map_used == 1):
          PHOTO2.append([child[0],child[1]])
          print(len(PHOTO1))
    child_no += 1

def recover_path(env, robotpos, targetpos):
  prev_pos = targetpos.copy()
  path = []

  while(((int(prev_pos[0]) == env.start[0])&(int(prev_pos[1]) == env.start[1]))==0):
    path.append(prev_pos)
    a = GRAPH[str([prev_pos[0],prev_pos[1]])].parent_id
    prev_pos = a.copy()

  return path

if __name__ == "__main__":
  
  pass

  robotstart = np.array([0, 2])
  targetstart = np.array([5, 3])
  map = np.loadtxt('maps/map0.txt')
  robotplanner(map,robotstart,targetstart,None)

  # # For Plotting the relation between epsilon and time taken for first iteration.
  # e = 1
  # k = 300
  # clock = np.zeros((k))
  # e_arr = np.zeros((k))
  # for i in tqdm(range(k)):
  #   e_arr[i] = e
  #   t = time.time()
  #   robotplanner(map,robotstart,targetstart,None)
  #   clock[i] = (time.time() - t)*1000
  #   e += 0.01
  
  # plt.title("Relation between epsilon and time taken for first iteration")
  # plt.xlabel("epsilon")
  # plt.ylabel("time for first iteration (ms)")
  # plt.plot(e_arr,clock)
  # plt.show(block = True)