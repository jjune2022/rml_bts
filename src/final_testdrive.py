import rospy
import math
import numpy as np
import gurobipy as gp
from gurobipy import *

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from camera_data.msg import ReferencePoses

# status dictionary
status_dict = {1: "loaded",
               2: "optimal",
               3: "infeasible",
               4: "infeasible and unbounded",
               5: "unbounded",
               6: "cut off",
               7: "iteration limit",
               8: "node limit",
               9: "time limit",
               10: "solution limit",
               11: "interrupted",
               12: "numeric",
               13: "suboptimal",
               14: "in progress",
               15: "user objective limit",
               16: "work limit",
               17: "memory limit"}

class MPCController:
    def __init__(self, hz=50, horizon=10):
        rospy.Subscriber("/odom", Odometry, self.odom_update)
        rospy.Subscriber('/ref_pos', ReferencePoses, self.waypoints_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.waypoints = []
        self.horizon = horizon
        self.hz = hz
        self.dt = 1/hz              # time step = 1/50 = 20ms
        self.odom_pose = None       # position x, y, z, orientation x, y, z, w
        self.odom_twist = None      # linear x, y, z, angular x, y, z
        self.min_vel = 0
        self.max_vel = 0.5
        self.min_ang = -0.175
        self.max_ang = 0.175
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta0 = 0.0
        self.transformed_points = None

    def odom_update(self, data):
        self.odom_pose = data.pose.pose
        self.odom_twist = data.twist.twist


    def waypoints_callback(self, data):
        self.waypoints = [(point.x, point.y) for point in data.points]
        self.x0, self.y0, self.theta0 = self.odom_pose.position.x, self.odom_pose.position.y, self.get_yaw_from_quaternion(self.odom_pose.orientation)
        self.transformed_points = self.transform_points(self.waypoints, self.x0, self.y0, self.theta0)
        self.run_mpc()


    def transform_points(self, points, x0, y0, theta0):
        transformed_points = []
        cos_theta = np.cos(theta0)
        sin_theta = np.sin(theta0)
        
        for point in points:
            dx = point[0] - x0
            dy = point[1] - y0
            
            x_prime = cos_theta * dx + sin_theta * dy
            y_prime = -sin_theta * dx + cos_theta * dy
            
            transformed_points.append((x_prime, y_prime))
            
        return transformed_points



    def run_mpc(self):
        if len(self.waypoints) < self.horizon:
            return
        
        # initial state
        
        # print("initial x, y, theta: ", x0, y0, theta0)
        # v0, omega0 = self.odom_twist.linear.x, self.odom_twist.angular.z
        # print("initial vel, ang_vel: ", v0, omega0)

        # define the optimization model
        m = gp.Model()
        m.Params.outputFlag = False
        
        # x variables
        x_vars = m.addVars(np.arange(self.horizon), vtype=GRB.CONTINUOUS, name="x")
        # y variables
        y_vars = m.addVars(np.arange(self.horizon), vtype=GRB.CONTINUOUS, name="y")
        # v variables
        vx_vars = m.addVars(np.arange(self.horizon), lb=0, ub=1.0, vtype=GRB.CONTINUOUS, name="v_x")
        vy_vars = m.addVars(np.arange(self.horizon), lb=-1.0, ub=1.0,vtype=GRB.CONTINUOUS, name="v_y")
        # omega variables
        omgx_vars = m.addVars(np.arange(self.horizon), lb=0, vtype=GRB.CONTINUOUS, name="omg_x")
        omgy_vars = m.addVars(np.arange(self.horizon), lb=0, vtype=GRB.CONTINUOUS, name="omg_y")

        # define the constraints
        # Constraint 1: set initial points
        cons1_1 = m.addConstr(x_vars[0] == self.x0)
        cons1_2 = m.addConstr(y_vars[0] == self.y0)
        # cons1_3 = m.addConstr(vx_vars[0] == self.vx_init)
        # cons1_4 = m.addConstr(vy_vars[0] == self.vy_init)
        # cons1_3 = m.addConstr(omgx_vars[0] == 0)
        # cons1_4 = m.addConstr(omgy_vars[0] == 0)
        # Constraint 2: dynamics
        cons2_1 = m.addConstrs(x_vars[h+1] == x_vars[h] + self.dt * vx_vars[h] for h in range(self.horizon - 1))
        cons2_2 = m.addConstrs(y_vars[h+1] == y_vars[h] + self.dt * vy_vars[h] for h in range(self.horizon - 1))
        cons2_3 = m.addConstrs(vx_vars[h+1] == vx_vars[h] + self.dt * omgx_vars[h] for h in range(self.horizon - 1))
        cons2_4 = m.addConstrs(vy_vars[h+1] == vy_vars[h] + self.dt * omgy_vars[h] for h in range(self.horizon - 1))
        
        # set objective function
        # m.setObjective(gp.quicksum((self.waypoints[h][0] - x_vars[h]) for h in range(self.horizon)) + gp.quicksum((self.waypoints[h][1] - y_vars[h]) for h in range(self.horizon)), GRB.MINIMIZE)
        m.setObjective(gp.quicksum((self.transformed_points[h][0] - x_vars[h])**2 for h in range(self.horizon)) + gp.quicksum((self.transformed_points[h][1] - y_vars[h])**2 for h in range(self.horizon)), GRB.MINIMIZE)

        
        m._xvars = x_vars
        m._yvars = y_vars
        m._vxvars = vx_vars
        m._vyvars = vy_vars
        m._omgxvars = omgx_vars
        m._omgyvars = omgy_vars
        m.optimize()

        # status
        print("Solved (%s)", status_dict[m.status])

        if m.status == 2:
            print("Objective value: ", m.ObjVal)
            x_vals = m.getAttr('x', x_vars)
            y_vals = m.getAttr('x', y_vars)
            vx_vals = m.getAttr('x', vx_vars)
            vy_vals = m.getAttr('x', vy_vars)
            print("Waypoints: ", self.transformed_points)
            print("v = ", np.sqrt(vx_vals[0]**2 + vy_vals[0]**2))
            print("theta = ",np.tanh(vy_vals[0] / (vx_vals[0] + 1e-6)))

            control_cmd = Twist()
            control_cmd.linear.x = vx_vals[0]
            for i in range(self.horizon):
                print("### horizon: ", i, " ###")
                print("x, y, vx, vy: ", x_vals[i], y_vals[i], vx_vals[i], vy_vals[i])
            control_cmd.angular.z = np.tanh(vy_vals[0] / (vx_vals[0] + 1e-6))

            self.pub.publish(control_cmd)
    

    def get_yaw_from_quaternion(self, q):
        """
        Convert a quaternion into yaw angle (in radians)
        """
        import tf.transformations
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2]


if __name__ == "__main__":
    hz = 50
    rospy.init_node("MPCController")
    node = MPCController(hz)
    rate = rospy.Rate(hz)   # 50 Hz
    while not rospy.is_shutdown():
        rate.sleep()
