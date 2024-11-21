
import rospy
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, atan2, sqrt
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
import pandas as pd
import scipy.linalg as la
import sys







# Variables for tracking data (similar to Pure Pursuit)
NZ = 3 # # of state vector z = x,y,yaw
NU = 1 # # of input vector u = v,w
hz = 50
dt = 1 / hz  # time step



num_waypoints = 9
car_wheel_base=0.463
L = car_wheel_base / 2

# test bench
linear_velocity = 2.0




class LQRController:
    def __init__(self, hz=50, Q=np.diag([1.0, 1.0, 1.0]), R=np.diag([0.01]), car_wheel_base=0.463,disable_signals=True):
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
        self.velocity = 1.5   # Set velocity
        self.Q = Q  # State cost matrix
        self.R = R  # Input cost matrix
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


        self.run_lqr()

    def run_lqr(self):
        if len(self.waypoints[:, 0]) < 2 or self.odom_pose is None:
            return

        # estimate the cross-track error
        closest_idx = 0
        min_dist = float('inf')

        # Find the closest waypoint ahead of the robot
        for i in range(len(self.waypoints[0,:])):
            dist = sqrt((self.x0 - self.waypoints[0, i]) ** 2 + (self.y0 - self.waypoints[1, i]) ** 2)
            if dist < min_dist:
                min_dist = dist
                #print()
                #print('mindist: ',min_dist)
                closest_idx = i
            #print(f'{i}:', self.waypoints[0, i])
            #print(f'{i}:', self.waypoints[1, i])


        # Compute the heading error
        path_angle = self.waypoints[2,5] # waypoint 6th(idx : 5) : yaw at nearest path

        # Set current state and state error
        x_actual = np.array([self.x0, self.y0, self.theta0])
        x_desired = np.array([self.waypoints[0, closest_idx + 1], self.waypoints[1, closest_idx + 1], path_angle])
        state_error = x_actual - x_desired

        # LQR controller


        A,B = get_linear_model_matrix(self.theta0)
        K = dlqr(A, B, self.Q, self.R)
        u = -K.dot(state_error)



        # clipped the control inputs
        #v = np.clip(u[0], -self.velocity, self.velocity)  # Limit velocity based on current settings
        v = self.velocity

        w = np.clip(u[0], -2.5235, 2.5235)  # Limit angular velocity




        # Publish control command
        control_cmd = Twist()
        control_cmd.linear.x = v
        control_cmd.angular.z = w
        self.pub.publish(control_cmd)


        #rospy.loginfo(f"Velocity: {v}, Angular Velocity: {w}, Closest Waypoint: {self.waypoints[:, closest_idx]}")



def get_yaw_from_quaternion(q):
    euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return euler[2]


def get_linear_model_matrix(phi): # phi = self.theta0
    A = np.zeros((NZ, NZ))
    A[0,0] = 1.0
    A[1,1] = 1.0
    A[2,2] = 1.0

    B = np.zeros((NZ, NU))
    B[2,0] = dt

    return A, B





def solve_DARE(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    P = Q
    Pn = Q
    max_iter = 150
    eps = 0.01

    for i in range(max_iter):
        Pn = Q + A.T @ P @ A - A.T @ P @ B @ \
            la.inv(B.T @ P @ B + R) @ B.T @ P @ A
        if (abs(Pn - P)).max() < eps:
            break
        P = Pn

    return Pn

def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    P = solve_DARE(A, B, Q, R)

    # compute the LQR gain
    K = la.inv(B.T @ P @ B + R) @ (B.T @ P @ A)


    return K



if __name__ == "__main__":

    hz = 50
    rospy.init_node("LQRController")
    node = LQRController(hz)


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
