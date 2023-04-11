#!/usr/bin/env python
import sys
import rospy
import tf2_msgs, geometry_msgs, tf2_ros
from rb5_control.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped, Twist
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion
import numpy as np
import csv

def update(apriltagarray):

    z = apriltagarray[0]
    markers_index = apriltagarray[1]
    z_id = apriltagarray[2]
    env_state = apriltagarray[3]

    mean       = env_state["mean"]
    covariance = env_state["covariance"]
    
    H = get_H(mean, z, len(z_id))
    print("H",H)
    
    V = np.eye(H.shape[0])*0.1

    S = np.matmul(np.matmul(H,covariance),H.T) + V

    K = np.matmul(np.matmul(covariance,H.T),np.linalg.inv(S))
    innovation = z - np.matmul(H,mean)
    mean = mean + np.matmul(K,(innovation))
    covariance = np.matmul((np.eye(K.shape[0]) - np.matmul(K,H)),covariance)

    env_state["mean"]       = list(mean)
    env_state["covariance"] = covariance
    
    return env_state


def get_H(state,obs,n_lm):
        th = -state[2]
        ret = np.zeros((len(obs),len(state)))
        for i in range(n_lm):
            if (obs[3*i]):
                ret[3*i,0] = -np.cos(th)
                ret[3*i,1] = np.sin(th)
                ret[3*i+1,0] = -np.sin(th)
                ret[3*i+1,1] = -np.cos(th)
                ret[3*i+2,2] = -1
            
        for i in range(n_lm):
            if (obs[3*i]):
                ret[i*3 , i*3 +3] = np.cos(th)
                ret[i*3 , i*3 +4] = -np.sin(th)
                
                ret[i*3 +1 , i*3 +3] = np.sin(th)
                ret[i*3 +1 , i*3 +4] = np.cos(th)
                
                ret[i*3+2 , i*3 + 5] = 1
        
        return ret 


if __name__ == "__main__":
    
    pass

