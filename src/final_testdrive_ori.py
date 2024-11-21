import rospy
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from camera_data.msg import ReferencePoses

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

    def odom_update(self, data):
        self.odom_pose = data.pose.pose
        self.odom_twist = data.twist.twist

    def waypoints_callback(self, data):
        # if self.odom_pose is None:
        #     rospy.logwarn("Odometry data has not been received yet.")
        #     return
        
        # self.waypoints = [(point.x, point.y) for point in data.points][::-1]
        self.waypoints = [(self.dt * np.sqrt(2) * point.x, self.dt * np.sqrt(2) * point.y) for point in data.points]

        self.run_mpc()

    def run_mpc(self):
        if len(self.waypoints) < self.horizon:
            return
        
        print(self.waypoints)
        # initial state
        x0, y0, theta0 = self.odom_pose.position.x, self.odom_pose.position.y, self.get_yaw_from_quaternion(self.odom_pose.orientation)
        print(x0, y0, theta0)
        v0, omega0 = self.odom_twist.linear.x, self.odom_twist.angular.z
        print(v0, omega0)
        # define variables
        x = cp.Variable(self.horizon)
        y = cp.Variable(self.horizon)
        theta = cp.Variable(self.horizon)
        v = cp.Variable(self.horizon) # linear velocity
        omega = cp.Variable(self.horizon) # angular velocity

        # Precompute cosines and sines
        cos_theta = np.cos(np.linspace(theta0, theta0 + (self.horizon - 1) * self.dt * omega0, self.horizon))
        sin_theta = np.sin(np.linspace(theta0, theta0 + (self.horizon - 1) * self.dt * omega0, self.horizon))

        # define the cost function
        cost = 0
        for t in range(self.horizon):
            cost += cp.square(x[t] - self.waypoints[t][0]) + cp.square(y[t] - self.waypoints[t][1])
        #     cost += cp.square(x[t] - self.waypoints[t][0]) + cp.square(y[t] - self.waypoints[t][1]) + cp.square(theta[t] - np.arctan2(self.waypoints[t][1] - y[t], self.waypoints[t][0] - x[t]))
        # for t in range(self.horizon):
        #     dx = self.waypoints[t][0] - x[t]
        #     dy = self.waypoints[t][1] - y[t]
        #     target_angle = cp.atan2(dy, dx)
        #     cost += cp.square(x[t] - self.waypoints[t][0]) + cp.square(y[t] - self.waypoints[t][1]) + cp.square(theta[t] - target_angle)

        # define the constraints
        constraints = []
        constraints += [x[0] == self.waypoints[0][0], y[0] == self.waypoints[0][1], theta[0] == theta0, v[0] == v0, omega[0] == omega0]
        for t in range(self.horizon - 1):
            constraints += [
                x[t + 1] == x[t] + self.dt * v[t] * np.cos(theta[t]),
                y[t + 1] == y[t] + self.dt * v[t] * np.sin(theta[t]),
                theta[t + 1] == theta[t] + self.dt * omega[t],
                v[t] >= self.min_vel,
                v[t] <= self.max_vel,
                omega[t] >= self.min_ang,
                omega[t] <= self.max_ang,
            ]
            constraints += [
                v[self.horizon - 1] >= self.min_vel,
                v[self.horizon - 1] <= self.max_vel,
                omega[self.horizon - 1] >= self.min_ang,
                omega[self.horizon - 1] <= self.max_ang,
            ]

        # solve the optimization problem
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(verbose=False)
        print("status: ", prob.status)

        # get the control input and publish
        if prob.status == cp.OPTIMAL:
            control_cmd = Twist()
            control_cmd.linear.x = v.value[1]
            control_cmd.angular.z = omega.value[1]
            
            # Print optimization results
            print("Optimization Status: ", prob.status)
            print("Final Cost: ", prob.value)
            
            print("Optimal Velocity (v): ", v.value)
            print("Optimal Angular Velocity (omega): ", omega.value)
            
            print("Optimal Path X: ", x.value)
            print("Optimal Path Y: ", y.value)
            print("Optimal Theta: ", theta.value)
            
            print("Reference X at t=1: ", self.waypoints[1][0])
            print("Reference Y at t=1: ", self.waypoints[1][1])
            
            self.pub.publish(control_cmd)


    def get_yaw_from_quaternion(self, q):
        """
        Convert a quaternion into yaw angle (in radians)
        """
        import tf.transformations
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2]


if __name__ == "__main__":
    hz = 10
    rospy.init_node("MPCController")
    node = MPCController(hz)
    rate = rospy.Rate(hz)   # 50 Hz
    while not rospy.is_shutdown():
        rate.sleep()