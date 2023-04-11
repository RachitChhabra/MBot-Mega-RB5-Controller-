import numpy as np
import matplotlib.pyplot as plt
from bresenham import bresenham

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

# def line(x0, y0, x1, y1):
#     deltax = x1-x0
#     dxsign = int(abs(deltax)/deltax)
#     deltay = y1-y0
#     dysign = int(abs(deltay)/deltay)
#     deltaerr = abs(deltay/deltax)
#     error = 0
#     y = y0
#     for x in range(x0, x1, dxsign):
#         yield x, y
#         error = error + deltaerr
#         while error >= 0.5:
#             y += dysign
#             error -= 1
#     yield x1, y1

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
                if mp[point[0],point[1]] == 1:
                    boom = 1
            
            if not boom:
                final_waypoints.append(end_point)
                found = 1
                start_point = end_point
            else:
                idx +=1

    return final_waypoints

# res = int(1/0.001)


if __name__ == "__main__":

    path = np.loadtxt("path100.txt")
    mp = np.loadtxt("map100.txt")

    # initial_waypoints = get_points_angle_change(path)

    # final_waypoints = get_final_waypoints(initial_waypoints, mp)

    a = np.zeros((4,4))
    a[2,2] = 5
    b = np.ones((4,4))


    plt.show(block = True)

