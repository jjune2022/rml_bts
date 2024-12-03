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
from collections import deque
from tf.transformations import euler_from_quaternion
import signal
from datetime import datetime
import pandas as pd


max_w = 0.7

num_waypoints = 9
car_wheel_base=0.463
L = car_wheel_base / 2

# test bench
linear_velocity = 1.0

class PurePursuitController:
    def __init__(self, hz=10, ld_gain=1.0, ld_min = 3):
        rospy.init_node('PurePursuitController')
        rospy.Subscriber("/odom", Odometry, self.odom_update)
        rospy.Subscriber('/way_points', Path, self.waypoints_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/lookahsead_marker', Marker, queue_size=10)
        self.hz = hz
        self.dt = 1 / hz  # time step
        self.odom_pose = None
        self.odom_twist = None
        self.x0 = 0.0
        self.y0 = 0.0
        self.ld_min =ld_min
        self.theta0 = 0.0
        self.ld_gain = ld_gain   # Lookahead distance gain
        self.car_wheel_base = car_wheel_base
        self.velocity = linear_velocity   # m/s
        self.waypoints = np.empty((3,0))

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

        self.run_pure_pursuit()

    def run_pure_pursuit(self):
        # if (np.linalg.norm(self.waypoints[:,5]- self.waypoints[:,6])) == 0.0:
        # # Publish control command
        #     control_cmd = Twist()
        #     control_cmd.linear.x = 0
        #     control_cmd.angular.z = 0
        #     self.pub.publish(control_cmd)
        #     rospy.loginfo('!!Finished!!')
        #     rospy.signal_shutdown()
        #     return


        ld = self.ld_gain * self.velocity+self.ld_min  # Lookahead distance


        # Compute the heading error
        path_angle = self.waypoints[2, 5] # waypoint 6th(idx : 5) : yaw at nearest path
        heading_error =path_angle - self.theta0

        #Normalize heading error to [-pi, pi]
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi



        # Find the lookahead point
        # Find the closest waypoint ahead of the robot
        lookahead_point_idx= int(ld//0.1)
        if num_waypoints < ld//0.1:
            lookahead_point = self.waypoints[:, -1]
        else:
            lookahead_point = self.waypoints[:, lookahead_point_idx]



        # Publish the lookahead point as a Marker
        self.publish_lookahead_marker(lookahead_point)

        # Calculate the steering angle
        alpha = atan2(lookahead_point[1] - (self.y0- L * sin(self.theta0)), lookahead_point[0] - ((self.x0 - L * cos(self.theta0) ))) - self.theta0
        steering_angle = atan2(2 * self.car_wheel_base * sin(alpha), ld)  # pure pursuit eq. !!!!!!!!!!!!!
        omega = steering_angle / self.dt
        omega = np.clip(omega, -max_w, max_w)  # Limit angular velocity

        # Publish control command
        control_cmd = Twist()
        control_cmd.linear.x = self.velocity
        control_cmd.angular.z = omega





        self.pub.publish(control_cmd)


        rospy.loginfo(f"Velocity: {self.velocity}, Steering Angle: {steering_angle}, pose: {self.waypoints[:,0]}, Lookahead Point: {lookahead_point}")


    def publish_lookahead_marker(self, lookahead_point):
        marker = Marker()
        marker.header.frame_id = "odom"  # Set the frame ID to your coordinate frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "lookahead_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = lookahead_point[0]
        marker.pose.position.y = lookahead_point[1]
        marker.pose.position.z = 0  # Assuming 2D navigation
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # Size of the sphere
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # Opacity
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0  # Blue
        self.marker_pub.publish(marker)

def get_yaw_from_quaternion(q):
    euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return euler[2]



if __name__ == "__main__":

    hz = 10
    rospy.init_node("PurePursuitController")
    node = PurePursuitController(hz)


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
