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
from update import update
from predict import predict_position

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
        self.maximumValue = 0.1

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


global ref
global stamp
global prev_stamp
global prev_state
global apriltagarray

markers_index = {}
global env_state
env_state = {"mean": [0,0,0],
             "covariance": np.zeros((3,3))}

def apriltag_callback(at_arr):
    global apriltagarray
    global env_state
    z = []
    z_id = {}
    if (len(at_arr.detections) > 0):
        for i in range(len(at_arr.detections)):
            x = at_arr.detections[i].pose.position.z
            y = -at_arr.detections[i].pose.position.x
            
            euler_angles = euler_from_quaternion([at_arr.detections[i].pose.orientation.x, at_arr.detections[i].pose.orientation.y, at_arr.detections[i].pose.orientation.z, at_arr.detections[i].pose.orientation.w])
            yaw = -euler_angles[1]
            yawd = yaw
            
            tag_robot = [x, y, yaw]
            
            robot_world = env_state["mean"][0:3]

            tag_world_T = robot_to_world_frame(tag_robot, robot_world)
            

            #wrapping angle to -pi/2 to pi/2
            tag_world = [tag_world_T[0,3], tag_world_T[1,3], tag_robot[2] + robot_world[2]]
            angle = tag_world[2]
            angle =  (angle + np.pi) % (2 * np.pi) - np.pi
            tag_world[2] = angle

            # print(at_arr.detections[i].id,tag_world,'world')
            z.append(tag_robot[0])
            z.append(tag_robot[1])
            z.append(tag_robot[2])

            z_id[at_arr.detections[i].id] = 1

            if at_arr.detections[i].id not in markers_index:
                markers_index[at_arr.detections[i].id] = len(markers_index) + 1
                increase_matrix_size(tag_world)
        apriltagarray = [z, markers_index, z_id, env_state]

def robot_to_world_frame(tag_robot, robot_world):
    robot_theta = -robot_world[2]

    rTa = np.array([[np.cos(tag_robot[2]), np.sin(tag_robot[2]), 0, tag_robot[0]],[-np.sin(tag_robot[2]), np.cos(tag_robot[2]), 0, tag_robot[1]], [0,0,1,0],[0,0,0,1]])
    wTr = np.array([[np.cos(robot_theta), np.sin(robot_theta), 0, robot_world[0]],[-np.sin(robot_theta), np.cos(robot_theta), 0, robot_world[1]], [0,0,1,0],[0,0,0,1]])

    tag_world = np.matmul(wTr,rTa)

    return tag_world

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

def listener():
    # rospy.Subscriber('tf', TFMessage, tf_callback)
    rospy.Subscriber('/apriltag_detection_array', AprilTagDetectionArray ,apriltag_callback, queue_size=20 )


if __name__ == "__main__":
    import time
    global apriltagarray
    global env_state
    ref = 0
    stamp = 0
    prev_stamp = 0
    prev_state = 0
    rospy.init_node("hw1")
    log = []
    # rate = rospy.Rate(0.1)
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    try:
        listener()
    except rospy.ROSInterruptException:
        pass

    waypoint = np.array([[1.0,0.0,0.0], 
                         [1.0,0.0,np.pi/2],
                         [1,1,np.pi/2],
                         [1,1,np.pi],
                         [0,1,np.pi],
                         [0,1,-np.pi/2],
                         [0,0,-np.pi/2],
                         [0.0,0.0,0.0]
                         ]) 
    #Octagon
    # waypoint = np.array([[0.4,-0.4,0],
    #                     [1.0,-0.4,0],
    #                     [1.0,-0.4,np.pi/4],
    #                     [1.4,0.0,np.pi/4],
    #                     [1.4,0.0,np.pi/2],
    #                     [1.4,0.6,np.pi/2],
    #                     [1.4,0.6,3*(np.pi/4)],
    #                     [1.0,1.0,3*(np.pi/4)],
    #                     [1.0,1.0,np.pi],
    #                     [0.4,1.0,np.pi],
    #                     [0.4,1.0,-3*(np.pi/4)],
    #                     [0.0,0.6,-3*(np.pi/4)],
    #                     [0.0,0.6,-np.pi/2],
    #                     [0.0,0.0,-np.pi/2],
    #                     # [0.0,0.0,0],
    #                     # [0.0,0.0,0.0], 
    #                     [0.0,0.0,-np.pi/4],
    #                     # [0.0,0.0,-np.pi/4],
    #                     [0.4,-0.4,-np.pi/4],]) 
                        # #  ])


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
    rospy.spin()