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
import cvxpy
# Variables for tracking data (similar to Pure Pursuit)
x_data = []
y_data = []
heading_error_data = []
cross_track_error_data = []
odom_x_for_heading_error = []
odom_x_for_cross_track_error = []
path_x_data = []
path_y_data = []
mpc_gain_values = []
mpc_vel = []
# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param
# config
car_wheel_base=0.463
NZ = 3 # # of state vector z = x,y,yaw
NU = 2 # # of input vector u = v,w
T = 5  # prediction horizon length
N_IND_SEARCH = 10  # Search index number
linear_velocity = 1.5 # Set constant velocity, !!!!!!!!!!!!!!if change the value, you should change the 'sp' on pathplanner node!!!!!!!!!!!!!!!!!!
hz = 50
dt = 1 / hz  # time step
max_w = 2.5235
# weight matrix
Q=np.diag([1.0, 1.0, 1.0])
Qf = Q
R=np.diag([0.01, 0.01])
Rd = np.diag([0.01,1.0])




class MPCController:

    def __init__(self, hz = hz, Q=np.diag([1.0, 1.0, 1.0]),
                  R=np.diag([0.01, 0.01]), Rd = np.diag([0.01,1.0]), max_w = max_w):
        rospy.init_node('MPCController')
        rospy.Subscriber("/odom", Odometry, self.odom_update)
        rospy.Subscriber('/way_points', Path, self.waypoints_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/lookahead_marker', Marker, queue_size=10)


        #state
        self.odom_pose = None
        self.odom_twist = None
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta0 = 0.0
        self.velocity = linear_velocity

        #mpc parameter
        self.Q = Q  # State cost matrix

        self.R = R  # Input cost matrix
        self.Rd = Rd # Input difference cost matrix
        self.waypoints = np.empty((3, 0))
        mpc_gain_values.append(self.Q)
        mpc_vel.append(self.velocity)

        self.Qf = Q  # state final matrix
        self.ov = None
        self.ow = None




        # callback rate
        self.hz = hz
        self.dt = 1 / hz  # time step

        # config
        self.car_wheel_base = car_wheel_base

        ### subject to
        # 1. linearized vehicle modle z_(t+dt) = A*z_(t) + B * u_t + C
        self.max_w = max_w # 2. Maximum angular velocity change
        # 3. Maximum angular velocity

    def odom_update(self, data):
        self.odom_pose = data.pose.pose
        self.odom_twist = data.twist.twist
        self.theta0 = get_yaw_from_quaternion(self.odom_pose.orientation)
        self.x0, self.y0 = self.odom_pose.position.x, self.odom_pose.position.y



    def waypoints_callback(self, data):

        for pose_stamped in data.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            yaw = pose_stamped.pose.orientation.w
            path = np.array([[x], [y], [yaw]])
            self.waypoints = np.concatenate((self.waypoints, path), axis=1)

        self.iterative_linear_mpc_control()

    def iterative_linear_mpc_control(self):
        """
        MPC control with updating operational point iteratively

        """


        if self.ov is None or self.ow is None:
            self.ov = get_nparray_from_matrix([0.0] * T)
            self.ow = get_nparray_from_matrix([0.0] * T)

        z0 = [self.x0, self.y0, self.theta0]
        for i in range(MAX_ITER):
            zbar = predict_motion(z0, self.ov, self.ow, self.waypoints)
            pow = self.ow[:]
            self.ov, self.ow, ox, oy, oyaw = linear_mpc_control(self.waypoints, z0, zbar)
            du = sum(abs(self.ow - pow))  # calc u change value
            if du <= DU_TH:
                break
        else:
            print("Iterative is max iter")



        # limitation of maximum input
        w = np.clip(self.ow[0], -self.max_w, self.max_w)  # Limit angular velocity
        v = self.velocity
        # Publish control command
        control_cmd = Twist()
        control_cmd.linear.x = v
        control_cmd.angular.z = w
        self.pub.publish(control_cmd)

        rospy.loginfo(f"Velocity: {v}, Angular Velocity: {w}, Closest Waypoint: {self.x0, self.y0}")


    def getB(self, theta, dt):
        B = np.array([[cos(theta) * dt, 0],
                      [sin(theta) * dt, 0],
                      [0, dt]])
        return B

    def compute_mpc_gain(self, Q, R, B):
        # Placeholder MPC gain computation; in practice, you'd solve the discrete-time algebraic Riccati equation (DARE)
        P = Q  # Assume simplified gain, P can be solved from Riccati equation
        K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P)
        return K
#################################################util############################3

# calculation for nearest index
def calc_nearest_index(state, cx, cy, cyaw, pind):
    """
    Simulation

    cx: course x position list
    cy: course y position list
    cy: course yaw position list
    cyaw: course yaw
    """
    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    # Upper of ref_path :positive
    angle = pi_2_pi(cyaw[ind] - atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind

# coordinate transformation
def get_yaw_from_quaternion(q):
    euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return euler[2]

# angle normailzation(-pi,pi)
def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def get_nparray_from_matrix(x):
    return np.array(x).flatten()

def predict_motion(z0, ov, ow, zref):
    #z0= x0,y0,yaw0

    zbar = zref * 0.0
    for i, _ in enumerate(z0):
        zbar[i, 0] = z0[i]

    state = State(x=z0[0], y=z0[1], yaw=z0[2])
    for (vi, wi, i) in zip(ov, ow, range(1, T + 1)):
        state = update_state(state, vi, wi)
        zbar[0, i] = state.x
        zbar[1, i] = state.y
        zbar[2, i] = state.yaw

    return zbar

class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None

def update_state(state, v, w):

    # input check
    if w>= max_w:
        delta = max_w
    elif w<= -max_w:
        delta = -max_w
    v = linear_velocity
    state.x = state.x + v* cos(state.yaw) * dt
    state.y = state.y + v* sin(state.yaw) * dt
    state.yaw = state.yaw + w * dt


    return state

#####################################################################3

# mpc
def linear_mpc_control(zref, z0, zbar):
    """
    linear mpc control

    zref: reference point
    zbar: operational point
    z0: initial state
    dref: reference steer angle
    """
    # run when # fo waypoints is more than one( # > 1 )
    if len(zref) < 2 or z0 is None:
        return

    z = cvxpy.Variable([NZ, T + 1]) # (t + 1, t + 1 + T)
    u = cvxpy.Variable([NU, T]) # (t + 1, t + T)

    cost = 0.0
    cost += cvxpy.quad_form(zref[:, T] - z[:, T], Qf) # cost for final state

    constraints = []
    for t in range(T):
        cost += cvxpy.quad_form(zref[:,t]-z[:,t], Q) #cost for state excepted final state
        cost += cvxpy.quad_form(u[:, t], R) # cost for input

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd) # cost for change of inputs

        # get_linear_model_matrix(w,phi,dt)
        A,B,C = get_linear_model_matrix(zbar[2, t])

        constraints += [z[:, t + 1] == A @ z[:, t] + B @ u[:, t] + C] # constraint 1

    constraints += [z[:, 0] == z0] # initial value
    constraints += [cvxpy.abs(u[1, :]) <= max_w] # constarint 2
    constraints += [cvxpy.abs(u[0, :]) <= linear_velocity] # constraint 3


    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.CLARABEL, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(z.value[0, :])
        oy = get_nparray_from_matrix(z.value[1, :])
        oyaw = get_nparray_from_matrix(z.value[2, :])
        ov = get_nparray_from_matrix(u.value[0, :])
        ow = get_nparray_from_matrix(u.value[1, :])
    else:
        print("Error: Cannot solve mpc..")
        ov, ow, ox, oy, oyaw = None, None, None, None, None, None

    return ov, ow, ox, oy, oyaw


# subject 1. linearized vehicle modle z_(t+dt) = A*z_(t) + B * u_t + C
def get_linear_model_matrix(phi): # phi = self.theta0
    A = np.zeros((NZ, NZ))
    A[0,0] = 1.0
    A[1,1] = 1.0
    A[2,2] = 1.0

    B = np.zeros((NZ, NU))
    B[0,0] = cos(phi) * dt
    B[1,0] = sin(phi) * dt
    B[2,1] = dt

    C = np.zeros((NZ, 1))

    return A, B, C











if __name__ == "__main__":

    hz = 50
    rospy.init_node("MPCController")
    node = MPCController(hz)


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
