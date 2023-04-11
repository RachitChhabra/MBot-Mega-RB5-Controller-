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
from update import update
from predict import predict_position
from coverage import get_waypoints_for_coverage

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
    hello = 1
    if (len(at_arr.detections) > 0):
        x = at_arr.detections[0].pose.position.z
        y = -at_arr.detections[0].pose.position.x
        marker_id = at_arr.detections[0].id
        eight = 0
        for i in range(len(at_arr.detections)):
            if(at_arr.detections[i].id == 9):
                eight = i
            elif(at_arr.detections[i].id == 8):
                eight = i
     

        for i in range(len(at_arr.detections)):
            
            markers_visible.append(at_arr.detections[i].id)
        
        euler_angles = euler_from_quaternion([at_arr.detections[0].pose.orientation.x, at_arr.detections[0].pose.orientation.y, at_arr.detections[0].pose.orientation.z, at_arr.detections[0].pose.orientation.w])
        
        yaw = -euler_angles[1]
        yawd = yaw

        tag_robot = [x, y, yaw]

        rTa = np.array([[np.cos(tag_robot[2]), np.sin(tag_robot[2]), 0, x],[-np.sin(tag_robot[2]), np.cos(tag_robot[2]), 0, y], [0,0,1,0],[0,0,0,1]])
        
        tag_world = marker[marker_id]
        
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

def increase_matrix_size(tag_world):
    env_state["mean"].append(tag_world[0])
    env_state["mean"].append(tag_world[1])
    env_state["mean"].append(tag_world[2])

    s = env_state["covariance"].shape[0]
    env_state["covariance"] = np.hstack((env_state["covariance"],np.zeros((s,3))))
    env_state["covariance"] = np.vstack((env_state["covariance"],np.zeros((3,s+3))))

    for i in range(env_state["covariance"].shape[0]):
        if((2*i - 1)%3 == 0):
            env_state["covariance"][i,i] == 0.1
        else:
            env_state["covariance"][i,i] == 0.01

    pass

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

def SLAM():
    global env_state
    waypoint = np.array([[1.0,0.0,0.0], 
                         [1.0,0.0,np.pi/2],
                         [1,1,np.pi/2],
                         [1,1,np.pi],
                         [0,1,np.pi],
                         [0,1,-np.pi/2],
                         [0,0,-np.pi/2],
                         [0.0,0.0,0.0]
                         ]) 
    pid = PIDcontroller(0.025,0.003,0.006)
    
    current_state = np.array([0.0,0.0,0.0])
    p = 0
    for wp in waypoint:
    
        print("move to way point", wp)
        
        pid.setTarget(wp)
        update_value = pid.update(current_state)
        pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
        
        time.sleep(0.2)
        current_state += update_value
        
        p+=1
        while((np.linalg.norm(pid.getError(current_state, wp)) > 0.1)): # check the error between current state and current way point


            update_value = pid.update(current_state)
            pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
        
            # PREDICT STEP
            env_state = predict_position(env_state, update_value)
            
            current_state = env_state["mean"][0:3]

            env_state = update(apriltagarray)
            #wrap angle
            env_state["mean"][2] =  (env_state["mean"][2] + np.pi) % (2 * np.pi) - np.pi
            
            # current_state = env_state["mean"][0:3]

            #sleep for giving it time to receive new sensor data
            time.sleep(0.2)
            pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
            time.sleep(0.5)

        pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
    return env_state
    
    
global env_state
env_state = {"mean": [0,0,0],
             "covariance": np.zeros((3,3))}


if __name__ == "__main__":
    import time
    global data
    global env_state

    rospy.init_node("hw1")

    SLAM_landmarks = SLAM()   ## To get landmarks from SLAM

    log = []
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    waypoint = get_waypoints_for_coverage()  ## To test the algorithm, using already built map.

    try:
        listener()
    except rospy.ROSInterruptException:
        pass

    robotstart = np.array([50 ,20])
    

    time.sleep(2)
    # init pid controller
 
    pid = PIDcontroller(0.02,0.003,0.006)
    # 0.03, 0.003, 0.006
    # init current state
    current_state = np.array([0.0,0.0,0.0])

    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough, 
    # the current way point will be updated with a new point.

    prev_wp = np.array([20,20,0])
    

    
    for wp_4 in waypoint:
        wp = [wp_4[0],wp_4[1],wp_4[2]]

        if wp_4[3] == 1:
            pid.maximumValue = 0.02
        elif wp_4[3] == 0:
            pid.maximumValue = 0.003


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

            # print(current_state)
            
            update_value = pid.update(current_state)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            time.sleep(0.2)
            # update the current state
            current_state += update_value

        pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
        time.sleep(0.5)

        prev_wp = wp

        # np.save("/data/data2.npy",np.asarray(log))
    
    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
    
    rospy.spin()

    

