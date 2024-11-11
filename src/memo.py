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
linear_velocity = 0.5 # Set constant velocity, !!!!!!!!!!!!!!if change the value, you should change the 'sp' on pathplanner node!!!!!!!!!!!!!!!!!!
hz = 50
dt = 1 / hz  # time step
max_w = 2.5235





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
        A,B,C = get_linear_model_matrix(
            u[1, t], zbar[2, t])

        constraints += [z[:, t + 1] == A @ z[:, t] + B @ u[:, t] + C] # constraint 1

    constraints += [z[:, 0] == z0] # initial value
    constraints += [cvxpy.ans(u[1, :]) <= max_w] # constarint 2
    constraints += [cvxpy.ans(u[0, :]) == linear_velocity] # constraint 3

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.CLARABEL, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        oyaw = get_nparray_from_matrix(x.value[2, :])
        ov = get_nparray_from_matrix(x.value[3, :])
        ow = get_nparray_from_matrix(u.value[0, :])
    else:
        print("Error: Cannot solve mpc..")
        ov, ow, ox, oy, oyaw = None, None, None, None, None, None

    return ov, ow, ox, oy, oyaw

def get_nparray_from_matrix(x):
    return np.array(x).flatten()

def iterative_linear_mpc_control(zref, z0, ov, ow):
    """
    MPC control with updating operational point iteratively

    """
    ox, oy, oyaw= None, None, None, None

    if ov is None or ow is None:
        ov = [0.0] * T
        ow = [0.0] * T

    for i in range(MAX_ITER):
        zbar = predict_motion(z0, ov, ow, zref)
        pow = ow[:]
        ov, ow, ox, oy, oyaw = linear_mpc_control(zref, z0, zbar)
        du = sum(abs(ow - pow))  # calc u change value
        if du <= DU_TH:
            break
    else:
        print("Iterative is max iter")

    return ov, ow, ox, oy, oyaw



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
