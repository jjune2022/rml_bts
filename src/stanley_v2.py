import os
import sys
import rospy
import numpy as np
import tf2_ros
import matplotlib.pyplot as plt
from math import cos, sin, tan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from collections import deque
from tf.transformations import euler_from_quaternion
import signal
from datetime import datetime
import pandas as pd


num_waypoints = 9
car_wheel_base=0.463
L = car_wheel_base / 2

# test bench
linear_velocity = 2.0







class Stanleycontroller:
    def __init__(self, hz=50, k=1.0):
        rospy.init_node('Stanleycontroller')
        rospy.Subscriber("/odom", Odometry, self.odom_update)
        rospy.Subscriber('/way_points', Path, self.waypoints_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.local_points = []
        self.global_points = None
        self.hz = hz
        self.dt = 1 / hz  # time step
        self.odom_pose = None  # position x, y, z, orientation x, y, z, w
        self.odom_twist = None  # linear x, y, z, angular x, y, z
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta0 = 0.0
        self.k = k            # Stanley controller gain for angle correction
        self.marker_id = 0
        self.angles = np.linspace(0, np.pi, 100)
        self.waypoints = []
        self.velocity = linear_velocity # m/s


    def odom_update(self, data):
        self.odom_pose = data.pose.pose
        self.odom_twist = data.twist.twist
        self.theta0 = get_yaw_from_quaternion(self.odom_pose.orientation)
        self.x0, self.y0 = self.odom_pose.position.x, self.odom_pose.position.y

    def waypoints_callback(self, data):
        self.waypoints = np.empty((3,0))
        for pose_stamped in data.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            yaw = pose_stamped.pose.orientation.w
            path = np.array([[x], [y], [yaw]])
            self.waypoints = np.concatenate((self.waypoints, path), axis=1)

        self.run_stanley()


    def run_stanley(self):
        if len(self.waypoints) < 2 or self.odom_pose is None:
            return

        v0 = self.velocity
        velocity = self.velocity

        # Compute the heading error
        path_angle = self.waypoints[2, 5] # waypoint 6th(idx : 5) : yaw at nearest path
        heading_error =path_angle - self.theta0

        #Normalize heading error to [-pi, pi]
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

        #y-y1 = (tan(path_angle) * x1) * (x- x1)
        #(tan(path_angle) * x1) * x -y +(-(tan(path_angle) * x1)* x1 + y1)
        # the car's tangent line below: cross_track negative / above: positive
        # estimate the cross-track error
        closest_idx = 0
        min_dist = float('inf')

        # Find the closest waypoint of the robot front wheels
        for i in range(len(self.waypoints[0,:])):
            dist = np.sqrt(((self.x0 + L * np.cos(self.theta0)) - self.waypoints[0, i]) ** 2 + ((self.y0  + L * np.sin(self.theta0)) - self.waypoints[1, i]) ** 2)
            if dist < min_dist:
                min_dist = dist
                #print()
                #print('mindist: ',min_dist)
                closest_idx = i
            #print(f'{i}:', self.waypoints[0, i])
            #print(f'{i}:', self.waypoints[1, i])




        # Waypoint to track is the closest one or next waypoint
        target_wp = self.waypoints[:, closest_idx]
        target_wp2 = self.waypoints[:, closest_idx+1]

        # Project RMS error onto front axle vector
        front_axle_vec = np.array([self.x0 - target_wp[0], self.y0 - target_wp[1]])
        nearest_path_vec = np.array([target_wp2[0]-target_wp[0], target_wp2[1]-target_wp[1]])
        cross_track_error = - np.sign(nearest_path_vec[0]*front_axle_vec[1] - nearest_path_vec[1]*front_axle_vec[0]) * min_dist


        print(cross_track_error)
        # Stanley control
        angle_correction = heading_error + np.arctan2(self.k * cross_track_error, self.velocity)
        angle_correction = (angle_correction + np.pi) % (2 * np.pi) - np.pi
        omega = angle_correction / self.dt  # /ref_pos 50초마다 publish
        omega = np.clip(omega, -2.5235, 2.5235)  # Limit angular velocity
        print(f'{heading_error} + {np.arctan2(self.k * cross_track_error, self.velocity)}')
        # Publish control command
        control_cmd = Twist()
        control_cmd.linear.x = self.velocity
        control_cmd.angular.z = omega
        self.pub.publish(control_cmd)


        #rospy.loginfo(f"Velocity: {velocity}, Angular Velocity: {omega}, Heading Error: {heading_error}, Path Distance: {cross_track_error}, Target: {target_wp}")




def get_yaw_from_quaternion(q):
    """
    Convert a quaternion into yaw angle (in radians)
    """
    euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return euler[2]

if __name__ == "__main__":

    hz = 50
    rospy.init_node("Stanleycontroller")
    node = Stanleycontroller(hz)


    rate = rospy.Rate(hz)
    #while not (rospy.is_shutdown() or KeyboardInterrupt):
    while not rospy.is_shutdown():
        try:
            # 노드가 동작 중일 때 주기적으로 sleep
            rate.sleep()
        except rospy.ROSInterruptException:
            print("ROS Interrupt Exception caught")
            break
        except KeyboardInterrupt:
            print("Keyboard Interrupt detected")
            rospy.signal_shutdown("KeyboardInterrupt received")
            break
