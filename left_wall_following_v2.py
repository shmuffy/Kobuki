#!/usr/bin/env python3

import roslib
roslib.load_manifest('ee106s24')
import rospy
import sys
import tf
import numpy as np
from math import pi, sqrt
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
import os


class PIDController:
    def __init__(self, P=1.0, I=0.0, D=0.1, set_point=0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.set_point = set_point # reference (desired value)
        self.previous_error = 0
        self.sum_error = 0

    def update(self, current_value):
        # calculate P_term and D_term
        #  e = r-y
        error = self.set_point - current_value 
        P_term = self.Kp * error
        D_term = self.Kd * (error - self.previous_error)
        
        self.sum_error += self.Ki * error  # I_term
        self.previous_error = error
        return P_term + self.sum_error + D_term

    def setPoint(self, set_point):  
        self.set_point = set_point
        self.previous_error = 0
    
    def setPD(self, P=0.0, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D

class Turtlebot():
    def __init__(self, goal_x, goal_y, csv_file):
        # Data to be taken from Launch File. Dont Edit
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.csv_file = csv_file
        
        # Initialize subscribers and publishers
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.vel_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)
        
        # Initialize state variables
        self.left = "O"
        self.front = "F"
        self.wall = "S"
        self.state = "forward"
        self.current_facing = 0  #straight
        self.logging_counter =0
        self.left_min_dist = 1                                          # Change if necessary
        self.forward_min_dist = 999
        self.pose = Pose2D()
        self.goal_tolerance = 0.3
        self.trajectory = list()
        self.angular_threshold = 0.003
        self.pd_control = PIDController()
        self.pd_control.setPD(P=0.3, D=0)
        
        # Added by me
        self.angle_error_thres = 0.03
        
        self.wall_controller = PIDController()
        self.wall_controller.setPD(P=1, D=5.2)
        
        # Change this value to change the behavior of the PD controller (see publish_velocity)
        self.ang_vel_scale = -1

        # For the complex world map
        self.control_list = [pi/2, pi]
        
        #Each right/left turn changes which ref angle we are using
        # ~ self.reference_angles = [-pi, -pi/2, 0, pi/2, pi]
        # ~ self.cur_ref = 2   # Robot starts at angle of 0 degrees

        # Define Finite-State Machine matrix by NumPy
        self.state_transition_matrix = np.array([
            #left Side: Free
            [   # Front side: free  -> Left
                [1],        
                # Front side: Occupied  -> Right
                [2] 
            ],
            #left Side: Occupied
            [   # Front side: free  -> Forward
                [0],
                # Front side: Occupied  -> Right
                [2]
            ]
        ])

        # Define state and condition encoding
        self.state_encoding = {'forward': 0, 'left': 1, 'right': 2}
        self.state_decoding = {0: 'forward', 1: 'left', 2: 'right'}
        self.condition_encoding = {'F': 0, 'O': 1}      #F: free; O:Occupied
        self.current_facing_decoding= {0:"straight", -1:"small_left", -2:"medium_left", -3:"large_left",
                                         1:"small_right",   2:"medium_right", 3:"large_right"}
        self.run()

    def run(self):
        # Don't edit anything
        while not rospy.is_shutdown():
            self.update_state() # Update the robot's state based on sensor readings
            # Publish velocity commands based on the current state
            self.publish_velocity() 
            # Sleep to maintain the loop rate
            self.rate.sleep()  

    def update_state(self):
        # State machine to update the robot's state based on sensor readings
        current_state_encoding = self.state_encoding[self.state]
        left_cond_encoding = self.condition_encoding[self.left]
        front_cond_encoding = self.condition_encoding[self.front]

        # Write code to encode current state and conditions
        # Get the new state from the state transition matrix
        new_state_encoded = self.state_transition_matrix[left_cond_encoding, front_cond_encoding][0]

        # Decode the new state
        self.state = self.state_decoding[new_state_encoded]
        
    def publish_velocity(self):
        vel = Twist()
        # Publish velocity commands based on the current facing direction
        # Fill in the velocities, keep the values small
        # Keep editing values in the given range till the robot moves well.

        vel.linear.x = 0.2  # Updated velocity

        if self.state == "right":
            print("+++++++++++ WE ARE TURNING RIGHT!")
            self.wall_controller.setPoint(0.9)
            
            if self.forward_min_dist != 999:
                vel.angular.z = self.ang_vel_scale * self.wall_controller.update(self.forward_min_dist)
                
            if vel.angular.z > 3.5:
                vel.angular.z = 3.5
            elif vel.angular.z < -3.5:
                vel.angular.z = -3.5
                
        # ~ elif self.state == "left":
            # ~ print("----------- WE ARE TURNING LEFT!")
            # ~ self.wall_controller.setPoint(0.)
            
            # ~ if self.forward_min_dist != 999:
                # ~ vel.angular.z = self.ang_vel_scale * self.wall_controller.update(self.pose.theta)
                
            # ~ if vel.angular.z > 3.5:
                # ~ vel.angular.z = 3.5
            # ~ elif vel.angular.z < -3.5:
                # ~ vel.angular.z = -3.5
        
            # ~ self.pd_control.setPoint(self.pose.theta % (pi/2))
            # ~ while abs(self.pose.theta % (pi/2)) > self.angular_threshold:
                # ~ vel.angular.z = -0.1 * self.pd_control.update(self.pose.theta % (pi/2))
                # ~ vel.linear.x = 0
                # ~ self.vel_pub.publish(vel)
                # ~ self.rate.sleep()
        else:
            print("WE ARE WALL HUGGING")
            self.wall_controller.setPoint(0.3)
            vel.angular.z = self.ang_vel_scale * self.wall_controller.update(self.left_min_dist)
            if vel.angular.z > 4:
                vel.angular.z = 4
            elif vel.angular.z < -4:
                vel.angular.z = -4
        
        self.vel_pub.publish(vel)
        
        # ~ if self.state == "left":
            # ~ print("------------------------------------ TURNING LEFT ------------------------------------")
            
            # use PD controller to control the angle: 
            # ~ self.pd_control.setPoint(self.angle_error_thres)
            # ~ while abs((self.pose.theta % (pi/2))) < self.angle_error_thres:
                # ~ vel.angular.z = self.ang_vel_scale * self.pd_control.update((self.pose.theta % (pi/2)))
                # ~ vel.linear.x = 0
                # ~ self.vel_pub.publish(vel)
                # ~ self.rate.sleep()
            # ~ print("done with turning")
            
            # ~ # move forward a bit
            # ~ for i in range(11):
                # ~ vel.linear.x = 0.2
                # ~ vel.angular.z = 0
                # ~ self.vel_pub.publish(vel)
                # ~ self.rate.sleep()

            # ~ # use PD controller to control the angle
            # ~ self.pd_control.setPoint(self.control_list[0])
            # ~ while abs(self.pose.theta - self.pd_control.set_point) >self.angular_threshold:
                # ~ vel.angular.z = self.pd_control.update(self.pose.theta)
                # ~ vel.linear.x = 0
                # ~ self.vel_pub.publish(vel)
                # ~ self.rate.sleep()
            # ~ print("done with turning")

            # ~ # move forward a bit: 
            # ~ for i in range(92):
                # ~ vel.linear.x = 0.2
                # ~ vel.angular.z = 0
                # ~ self.vel_pub.publish(vel)
                # ~ self.rate.sleep()
        
        # ~ elif self.state == "right":
            # ~ print("------------------------------------ TURNING RIGHT ------------------------------------")
            # ~ self.pd_control.setPoint(self.pose.theta - pi/2)
            
            # ~ while abs(self.pose.theta - self.pd_control.set_point) > self.angular_threshold:
                # ~ vel.angular.z = self.pd_control.update(self.pose.theta)
                # ~ vel.linear.x = 0
                # ~ self.vel_pub.publish(vel)
                # ~ self.rate.sleep()
            # ~ print("done with right turing")
            
        # ~ else: # Wall hugging mode
            # ~ self.wall_controller.setPoint(0.3)
            
            # ~ vel.angular.z = self.ang_vel_scale * self.wall_controller.update(self.left_min_dist)
            # ~ if vel.angular.z > 2.5:
                # ~ vel.angular.z = 2.5
            
            # ~ vel.linear.x = 0.2
            
            # ~ print("self.left_min_dist = ", self.left_min_dist)
            # ~ print("vel.angular.z = ", vel.angular.z)
            # ~ self.vel_pub.publish(vel)

    def lidar_callback(self, data):
        self.left_min_dist=1                                            # Change if necessary
        
        # Update the forward distance with the distance directly in front of the robot
        if str(data.ranges[0]) == "inf":
            self.forward_min_dist = 999
        else:
            self.forward_min_dist = data.ranges[0]

        # transform the lidar points frame /rplidar_link from to another frame:
        listener = tf.TransformListener()
        (trans,rot) = listener.lookupTransform('/rplidar_link', '/cliff_sensor_left_link', rospy.Time(0))

        # Process the LIDAR data and transform the points to the robot's coordinate frame (another frame you specified)
        for i in range(len(data.ranges)):
            # get the left side lidar data
            # 2.1 and 1.037 are decent with P=1 and D=5.2
            if i * data.angle_increment < 2.1 and i * data.angle_increment > 1.037: # if i * data.angle_increment < 1.59 and i * data.angle_increment > 1.55:
                if str(data.ranges[i])=="inf":
                    dist = 9999
                else:
                    dist = data.ranges[i]   
                (x, y) = self.calculate_position_of_range(dist, i, data.angle_increment, data.angle_min)
                
                # See rangescheck_jackal.py
                rot_mat = tf.transformations.quaternion_matrix(rot)
                rot_mat[0,3] = trans[0]
                rot_mat[1,3] = trans[1]
                rot_mat[2,3] = trans[2]
                
                point = [x, y, 0, 1]
                transformed_point = np.dot(rot_mat, point) # 4-dimensional
                
                left_dist = sqrt(transformed_point[0]**2 + transformed_point[1]**2)
                
                # ~ print("left_dist = ", left_dist)
                # ~ print("self.left_min_dist = ", self.left_min_dist)
                # Feel free to comment out ################################################################################
                if left_dist < self.left_min_dist:
                    self.left_min_dist = left_dist
                    
                # keep the minimum distance as the left_min_dist
                # ~ self.left_min_dist = self.left_min_dist

        # Update left and forward state
        if self.left_min_dist < 0.4:
            self.left = "O"
        else:
            self.left = "F"
            
        if self.forward_min_dist < 0.8:
            self.front = "O"
        else:
            self.front = "F"

        # Set wall state
        if self.left_min_dist < 0.3:
            self.wall = "C"
        else:
            self.wall = "S"

        # Update current_facing direction
        # The basic idea is:
        #  if the robot is too close, take the medium_right facing; if it is not that close, then take the small_right facing
        #  if the robot is too far, take the medium_left facing; if it is not that far, then take the small_left facing
        #  
        # ~ if self.left_min_dist < :
            # ~ self.current_facing = 
        # ~ elif self.left_min_dist < :
            # ~ self.current_facing = 
        # ~ elif (self.left_min_dist > ) and (self.left_min_dist < ) :
            # ~ self.current_facing = 
        # ~ elif (self.left_min_dist >= ) and (self.left_min_dist < ) :
            # ~ self.current_facing = 
        # ~ elif (self.left_min_dist >= ):
            # ~ self.current_facing = 
        # ~ else:
            # ~ self.current_facing = 0
            
        # For display or non-PID controlled turning purposes only
        if self.left_min_dist <= 0.2:
            self.current_facing = 2   # medium_right
        elif 0.2 < self.left_min_dist <= 0.22:
            self.current_facing = 1   # small_right
        elif 0.22 < self.left_min_dist <= 0.24:
            self.current_facing = -1  # small_left
        elif self.left_min_dist > 0.24:
            self.current_facing = -2  # medium_left
        else:
            self.current_facing = 0   # straight
            

    def dist_goal(self):
        return sqrt((self.pose.x - self.goal_x)**2 + (self.pose.y - self.goal_y)**2)

    def calculate_position_of_range(self, range, idx, angle_increment, angle_min):
        if str(range) == "inf":
            rospy.loginfo("The provided range is infinite!")
            return -1
        theta = idx * angle_increment + angle_min
        x = range * np.cos(theta)
        y = range * np.sin(theta)
        return x, y
    
    def save_trajectory(self):
        # Save the trajectory to a CSV file, csv file name is given in launch file. Nothing to edit here
        np.savetxt(self.csv_file, np.array(self.trajectory), fmt='%f', delimiter=',')

    def odom_callback(self, msg):
        # Callback function to handle incoming odometry data
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, \
                       msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # Write code heck if the robot has reached the goal within the tolerance
        # if robot has reached goal, save the trajectory by self.save_trajectory(), and then shutdown ROS using rospy.signal_shutdown()
        
        if self.dist_goal() < self.goal_tolerance:
            self.save_trajectory()
            print("WE ARE DONE!")
            rospy.signal_shutdown('Yay goal reached! Saving CSV file...')


        # Log the odometry data every 100 iterations
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y]) 


def main(args):
    # Initialize ROS Node
    rospy.init_node('left_wall_follower', anonymous=True)
    # Get parameters from the launch file
    
    goal_x = rospy.get_param('goal_x')
    goal_y = rospy.get_param('goal_y')
    csv_file = rospy.get_param('csv_file')

    # Create an instance of the Turtlebot class
    robot = Turtlebot(goal_x, goal_y, csv_file)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
