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
import math
from astar import plan_path
# from spatialmath import *

"""
The class of the pid controller.
"""
class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.01

    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array([targetx, targety, targetw])

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array(state)

    def getError(self, currentState, targetState):
        """
        return the different between two states
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        
        return result 

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep 
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        # scale down the twist if its norm is more than the maximum value. 
        resultNorm = np.linalg.norm(result)
        if(resultNorm > self.maximumValue):
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result

def getError(currentState, targetState):
    """
    return the different between two states
    """
    currentState = np.array(currentState)
    targetState = np.array(targetState)
    result = targetState - currentState
    result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
    
    return result 

def genTwistMsg(desired_twist):
    """
    Convert the twist to twist msg.
    """
    twist_msg = Twist()
    twist_msg.linear.x = desired_twist[0] 
    twist_msg.linear.y = desired_twist[1] 
    twist_msg.linear.z = 0
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = desired_twist[2]
    return twist_msg

def coord(twist, current_state):
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)


data = {
        "stamp": 0,
        "prev_stamp":0,
        "current_state": [0.5,0.5,0],
        "prev_state" : [0.5,0.5,0]
        }

def listener():
    rospy.Subscriber('/apriltag_detection_array', AprilTagDetectionArray ,apriltag_callback, queue_size=20 )

def get_current_state():
    new_data = False

    global data

    if((data["prev_stamp"] != data["stamp"])):
        new_data = True

        data["prev_stamp"] = data["stamp"]

        data["prev_state"] = data["current_state"]
    else:
        data["current_state"] = data["prev_state"]

    return data["current_state"], new_data

def apriltag_callback(at_arr):

    global apriltagarray
    global data
    global marker_id
    markers_visible = []

    if (len(at_arr.detections) > 0):
        x = at_arr.detections[0].pose.position.z
        y = -at_arr.detections[0].pose.position.x
        marker_id = at_arr.detections[0].id

        for i in range(len(at_arr.detections)):
            
            markers_visible.append(at_arr.detections[i].id)
        
        euler_angles = euler_from_quaternion([at_arr.detections[0].pose.orientation.x, at_arr.detections[0].pose.orientation.y, at_arr.detections[0].pose.orientation.z, at_arr.detections[0].pose.orientation.w])

        yaw = -euler_angles[1]
        yawd = yaw

        tag_robot = [x, y, yaw]

        rTa = np.array([[np.cos(tag_robot[2]), np.sin(tag_robot[2]), 0, x],[-np.sin(tag_robot[2]), np.cos(tag_robot[2]), 0, y], [0,0,1,0],[0,0,0,1]])
        
        tag_world = marker[marker_id]
        print(markers_visible, marker_id)
        if len(tag_world) == 3:

            x = tag_world[0]
            y = tag_world[1]
            yaw = tag_world[2]

            wTa = np.array([[np.cos(tag_world[2]), np.sin(tag_world[2]), 0, x],[-np.sin(tag_world[2]), np.cos(tag_world[2]), 0, y], [0,0,1,0],[0,0,0,1]])
            
            wTr = np.matmul(wTa,np.linalg.inv(rTa))
            state = [0,0,0]
            state[0] = wTr[0,3]
            state[1] = wTr[1,3]
            state[2] = -yawd + yaw

            state[2] = -state[2]

        else:
            
            x = tag_world[0][0]
            y = tag_world[0][1]
            yaw = tag_world[0][2]

            wTa = np.array([[np.cos(tag_world[0][2]), np.sin(tag_world[0][2]), 0, x],[-np.sin(tag_world[0][2]), np.cos(tag_world[0][2]), 0, y], [0,0,1,0],[0,0,0,1]])
                
            wTr = np.matmul(wTa,np.linalg.inv(rTa))
            state1 = [0,0,0]
            state1[0] = wTr[0,3]
            state1[1] = wTr[1,3]
            state1[2] = -yawd + yaw

            state1[2] = -state1[2]

            #------------------------------------------------------------#

            x = tag_world[1][0]
            y = tag_world[1][1]
            yaw = tag_world[1][2]

            wTa = np.array([[np.cos(tag_world[1][2]), np.sin(tag_world[1][2]), 0, x],[-np.sin(tag_world[1][2]), np.cos(tag_world[1][2]), 0, y], [0,0,1,0],[0,0,0,1]])
                
            wTr = np.matmul(wTa,np.linalg.inv(rTa))
            state2 = [0,0,0]
            state2[0] = wTr[0,3]
            state2[1] = wTr[1,3]
            state2[2] = -yawd + yaw

            state2[2] = -state2[2]

            #------------------------------------------------------------#

            if (np.linalg.norm(getError(state1, data["prev_state"])) < np.linalg.norm(getError(state2, data["prev_state"]))):
                state = state1
                print("1")
            else:
                state = state2
                print("2")
        data["current_state"] = state
        data["prev_state"] = state
        data["stamp"] = at_arr.detections[0].header.stamp.secs

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
    vector_angle = math.acos(vector[0]/l2_dist)
    
    wp.append([current_wp[0]*lc,current_wp[1]*lc,vector_angle,1])
    wp.append([next_wp[0]*lc,next_wp[1]*lc,vector_angle,0])

  return wp


marker = {
                0:np.array([0.0,-0.135,np.pi/2 -1e-4]),
                7:np.array([2.55,0.0,0]),
                5:np.array([[-0.155,0.0,np.pi-1e-3],[1.15,1.05,np.pi/2 -1e-4]]),
                3:np.array([[2.43,-0.135,np.pi/2 -1e-4],[0.95,1.22,0]]),
                1:np.array([-0.12,2.44,np.pi-1e-3]),
                4:np.array([[0.0,2.58,-np.pi/2+1e-4],[1.245,1.215,-np.pi +1e-4]]),
                8:np.array([[2.58,2.44,0],[1.15,1.05,-np.pi/2+ 1e-4]]),
                9:np.array([2.58,2.58,-np.pi/2 +1e-4])

            }

if __name__ == "__main__":
    import time
    global data

    rospy.init_node("hw1")

    log = []
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    try:
        listener()
    except rospy.ROSInterruptException:
        pass

    robotstart = np.array([20 ,20])
    targetpos  = np.array([80 ,80])
    m          = "min_distance"                    # 'min_distance' -> 1 or 'max_safety' -> 0

    waypoint_2 = plan_path(robotstart, targetpos, m)

    if(m == 'min_distance'):
        waypoint = add_dimension(waypoint_2)
    else:
        waypoint = add_dimension(waypoint_2)
    print(waypoint)

    time.sleep(2)
    # init pid controller
 
    pid = PIDcontroller(0.02,0.003,0.006)
    
    # init current state
    current_state = np.array([0.5,0.5,0.0])

    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough, 
    # the current way point will be updated with a new point.
    
    for wp_4 in waypoint:
        wp = [wp_4[0],wp_4[1],wp_4[2]]

        if(m == 'min_distance'):
            if wp_4[3] == 1:
                pid.maximumValue = 0.05
            elif wp_4[3] == 0:
                pid.maximumValue = 0.003
            elif wp_4[3] == 0.8:
                pid.maximumValue = 0.028
        else:
            if wp_4[3] == 1:
                pid.maximumValue = 0.035
            elif wp_4[3] == 0:
                pid.maximumValue = 0.004

        print("move to way point", wp)

        # set wp as the target point
        pid.setTarget(wp)

        # calculate the current twist
        update_value = pid.update(current_state)

        # publish the twist
        pub_twist.publish(genTwistMsg(coord(update_value, current_state)))

        time.sleep(0.05)
        # update the current state
        current_state += update_value

        while((np.linalg.norm(pid.getError(current_state, wp)) > 0.12)): # check the error between current state and current way point

            # calculate the current twist

            current_state_visual, new_info = get_current_state()
            
            if (new_info is True):
                current_state = current_state_visual
                print(current_state)

            update_value = pid.update(current_state)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            time.sleep(0.2)
            # update the current state
            current_state += update_value

        pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
        time.sleep(0.5)


    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
    
    rospy.spin()

    

