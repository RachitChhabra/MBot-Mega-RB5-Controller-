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

    # We need a 0.5 meter obstacle in the center
    # 0.5 meter is 0.5/lc cells.

    obstacle = int(0.31/lc)  # 12 inches
    center = int(map.shape[0]/2)


  ### To test obstacles

    # for i in range(center - int(obstacle/2),center + int(obstacle/2)):
    #     map[i,center - int(obstacle/2):center + int(obstacle/2)] = np.ones((obstacle))

    # for i in range(40):
    #         map[i ,-40+i:] = 0.99
    #         map[-i,-40+i:] = 0.99

    # Minkowski Sum

    robot_width = int(0.15/lc) # in number of cells
    
    map[map == 0.99] = 1
    # np.savetxt("map.txt",map)

    return map
 
# Function to check if it is safe to go to position (x, y)
# from the current position. The function returns false if (x, y)
# is not valid matrix coordinates or (x, y) represents water or
# position (x, y) is already processed.
 
def isSafe(mat, x, y, processed):
    return (x >= 0 and x < len(processed)) and (y >= 0 and y < len(processed[0])) and \
           mat[x][y] == 1 and not processed[x][y]


if __name__ == "__main__":
    # you should change the following line to test different maps
    # map = np.loadtxt('map100m.txt')
    map = get_map()

    points = []
    ct = 0
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            ct+=1
            if(map[i,j] != 0):
                points.append([i,j])


    plt.imshow(map,cmap='Greys')
    plt.xlabel("x coodinate")
    plt.ylabel("y coodinate")
    plt.show()