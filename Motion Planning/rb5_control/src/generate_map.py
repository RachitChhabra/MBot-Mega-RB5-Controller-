import numpy as np
import matplotlib.pyplot as plt

def get_map():

    lc = 0.0245 # meters

    side = 2.45 # meters

    no_of_points = int(side/lc)

    map = np.zeros((no_of_points,no_of_points))

    map[0,:]  = 1
    map[-1,:] = 1
    map[:,0]  = 1
    map[:,-1] = 1
    print("")
    print("-------------------------------------------------")
    print('Map Size: ',map.shape)
    # We need a 0.5 meter obstacle in the center
    # 0.5 meter is 0.5/lc cells.

    obstacle = int(0.31/lc)  # 14 inches
    center = int(map.shape[0]/2)

    for i in range(center - int(obstacle/2),center + int(obstacle/2)):
        map[i,center - int(obstacle/2):center + int(obstacle/2)] = np.ones((obstacle))

    # Minkowski Sum

    robot_width = int(0.1/lc) # in number of cells

    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            if(map[i,j] == 1):
                for k in range(max(i-robot_width,0), min(i+robot_width+1,map.shape[0])):
                    for l in range(max(j-robot_width,0), min(j+robot_width+1,map.shape[1])):
                        map[k,l] = max(map[k,l],0.4)
    print("-------------------------------------------------")
    # np.savetxt("map.txt",map)
    return map

if __name__ == "__main__":
    # you should change the following line to test different maps

    # map = np.loadtxt('map100m.txt')
    map = get_map()
    plt.matshow(map)
    plt.show()