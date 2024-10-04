import os
import sys
import rospy
import numpy as np
import tf2_ros
import matplotlib.pyplot as plt
from math import cos, sin, atan2, sqrt
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
import signal
from datetime import datetime
import pandas as pd

# Variables for tracking data (similar to Pure Pursuit)
x_data = []
y_data = []
heading_error_data = []
cross_track_error_data = []
odom_x_for_heading_error = []
odom_x_for_cross_track_error = []
path_x_data = []
path_y_data = []
lqr_gain_values = []
lqr_vel = []

class LQRController:
    def __init__(self, hz=50, Q=np.diag([1.0, 1.0, 1.0]), R=np.diag([0.01, 0.01]), car_wheel_base=0.463,disable_signals=True):
        rospy.init_node('LQRController')
        rospy.Subscriber("/odom", Odometry, self.odom_update)
        rospy.Subscriber('/way_points', Path, self.waypoints_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/lookahead_marker', Marker, queue_size=10)

        self.hz = hz
        self.dt = 1 / hz  # time step
        self.odom_pose = None
        self.odom_twist = None
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta0 = 0.0
        self.car_wheel_base = car_wheel_base
        self.velocity = 0.5   # Set velocity
        self.Q = Q  # State cost matrix
        self.R = R  # Input cost matrix
        self.waypoints = []
        lqr_gain_values.append(self.Q)
        lqr_vel.append(self.velocity)

    def odom_update(self, data):
        self.odom_pose = data.pose.pose
        self.odom_twist = data.twist.twist
        self.theta0 = self.get_yaw_from_quaternion(self.odom_pose.orientation)
        self.x0, self.y0 = self.odom_pose.position.x, self.odom_pose.position.y

        x_data.append(self.x0)
        y_data.append(self.y0)

    def waypoints_callback(self, data):
        if len(self.waypoints) < 2:
            for pose_stamped in data.poses:
                x = pose_stamped.pose.position.x
                y = pose_stamped.pose.position.y
                self.waypoints.append((x, y))
                path_x_data.append(x)
                path_y_data.append(y)
        self.run_lqr()

    def run_lqr(self):
        if len(self.waypoints) < 2 or self.odom_pose is None:
            return

        # Find the closest waypoint ahead of the robot
        closest_idx = 0
        min_dist = float('inf')

        for i in range(len(self.waypoints)):
            dist = sqrt((self.x0 - self.waypoints[i][0]) ** 2 + (self.y0 - self.waypoints[i][1]) ** 2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        cross_track_error_data.append(min_dist)

        # Heading error
        path_angle = np.arctan2(self.waypoints[closest_idx + 1][1] - self.waypoints[closest_idx][1],
                                self.waypoints[closest_idx + 1][0] - self.waypoints[closest_idx][0])  # rad
        heading_error = path_angle - self.theta0
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi  # Normalize
        heading_error_data.append(heading_error)
        odom_x_for_heading_error.append(self.x0)
        odom_x_for_cross_track_error.append(self.x0)

        # Set current state and state error
        x_actual = np.array([self.x0, self.y0, self.theta0])
        x_desired = np.array([self.waypoints[closest_idx][0], self.waypoints[closest_idx][1], path_angle])
        state_error = x_actual - x_desired

        # LQR controller
        B = self.getB(self.theta0, self.dt)
        K = self.compute_lqr_gain(self.Q, self.R, B)
        u = -K.dot(state_error)

        v = np.clip(u[0], -self.velocity, self.velocity)  # Limit velocity based on current settings
        w = np.clip(u[1], -2.5235, 2.5235)  # Limit angular velocity
        v = self.velocity
        # Publish control command
        control_cmd = Twist()
        control_cmd.linear.x = v
        control_cmd.angular.z = w
        self.pub.publish(control_cmd)

        rospy.loginfo(f"Velocity: {v}, Angular Velocity: {w}, Closest Waypoint: {self.waypoints[closest_idx]}")

        if abs(self.x0 - path_x_data[-1]) < 0.5 and abs(self.y0 - path_y_data[-1]) < 0.5:
            rospy.loginfo("Reached the target position. Shutting down the node.")
            save_results_and_shutdown()

    def getB(self, theta, dt):
        B = np.array([[cos(theta) * dt, 0],
                      [sin(theta) * dt, 0],
                      [0, dt]])
        return B

    def compute_lqr_gain(self, Q, R, B):
        # Placeholder LQR gain computation; in practice, you'd solve the discrete-time algebraic Riccati equation (DARE)
        P = Q  # Assume simplified gain, P can be solved from Riccati equation
        K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P)
        return K

    def get_yaw_from_quaternion(self, q):
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2]

def save_results_and_shutdown():
    # Save results (same as pure pursuit logic)
    pass

def signal_handler(sig, frame):
    save_results_and_shutdown()

if __name__ == "__main__":

    hz = 50
    rospy.init_node("LQRController")
    node = LQRController(hz)

    signal.signal(signal.SIGINT, signal_handler)

    rate = rospy.Rate(hz)
    #while not (rospy.is_shutdown() or KeyboardInterrupt):
    while True:
        try:
            rate.sleep()
        except rospy.is_shutdown or KeyboardInterrupt:
            break

    save_results_and_shutdown()
