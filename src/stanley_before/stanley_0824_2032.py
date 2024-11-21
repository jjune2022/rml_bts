import os
import sys
import rospy
import numpy as np
import tf2_ros
from math import cos, sin
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry , Path
from visualization_msgs.msg import Marker
from collections import deque
from tf.transformations import euler_from_quaternion

x_data = []
y_data = []
fig, ax = plt.subplots()

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


class Stanleycontroller:
    def __init__(self, hz=50, k=0.3):
        rospy.init_node('Stanleycontroller')
        rospy.Subscriber("/odom", Odometry, self.odom_update)
        rospy.Subscriber('/way_points', Path, self.waypoints_callback)
        # self.marker_pub = rospy.Publisher('/transformed_points_marker', Marker,queue_size=10)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.local_points = []
        self.global_points = None
        # self.waypoints = []
        self.hz = hz
        self.dt = 1 / hz  # time step
        self.odom_pose = None  # position x, y, z, orientation x, y, z, w
        self.odom_twist = None  # linear x, y, z, angular x, y, z
        self.x0 = 0.0
        self.y0 = 0.0
        self.k = k            # Stanley controller gain for angle correction
        self.min_vel = 0.0
        self.marker_id = 0
        self.max_vel = 5
        self.angles = np.linspace(0, np.pi, 100)
        self.waypoints = []
        self.velocity = 1 # m/s

    # odom은 bringup으로부터 자동으로 publish됨(global좌표)
    def odom_update(self, data):
        self.odom_pose = data.pose.pose
        self.odom_twist = data.twist.twist
        self.theta0 =self.get_yaw_from_quaternion(self.odom_pose.orientation)
        self.x0, self.y0= self.odom_pose.position.x + 0.175 * cos(self.theta0), self.odom_pose.position.y +0.175 * sin(self.theta0)
        x_data.append(self.x0)
        y_data.append(self.y0)
    def waypoints_callback(self, data):
        # self.global_points = self.our_points[self.marker_id: self.marker_id + self.horizon]
        # self.local_points = self.global2local(self.global_points, self.x0, self.y0, self.theta0)
        # self.publish_global_points() #global point 표시
        if len(self.waypoints) < 2:
            for pose_stamped in data.poses:
                x = pose_stamped.pose.position.x
                y = pose_stamped.pose.position.y
                self.waypoints.append((x,y))
        self.run_stanley()



    def run_stanley(self):
        if len(self.waypoints) < 2 or self.odom_pose is None:
            return

        v0 = self.velocity
        velocity = self.velocity

        # Find the closest waypoint
        min_dist = float('inf')
        closest_idx = 0
        for i, (wx, wy) in enumerate(self.waypoints):
            dist = np.sqrt((self.x0 - wx)**2 + (self.y0 - wy)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Waypoint to track is the closest one or next waypoint
        target_wp = self.waypoints[closest_idx]

        # Compute the heading error
        path_angle = np.arctan2(target_wp[1] - self.y0, target_wp[0] - self.x0) #rad
        path_distance = np.sqrt((target_wp[0] - self.x0) ** 2 + (target_wp[1] - self.y0) ** 2)
        heading_error = path_angle - self.theta0

        # Normalize heading error to [-pi, pi]
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

        # Stanley control
        angle_correction = heading_error + np.arctan2(self.k * path_distance, v0)
        # angular velocity pet second
        # omega = angle_correction /self.dt
        omega = angle_correction / 1 #/ref_pos 1초마다 publish한다고 가정
        omega = np.clip(omega, -3, 3)  # Limit angular velocity

        # Control effort
        # velocity = np.clip(v0, self.min_vel, self.max_vel)

        # Publish control command
        control_cmd = Twist()
        control_cmd.linear.x = velocity
        control_cmd.angular.z = omega
        self.pub.publish(control_cmd)

        # Debug info
        rospy.loginfo(f"Velocity: {velocity}, Angular Velocity: {omega}, Heading Error: {heading_error}, Path Distanc: {path_distance}Target: {target_wp}")


    # def publish_way_points(self):
    #     """
    #     Publish the transformed points to RViz as a Marker
    #     """
    #     marker = Marker()
    #     marker.header.frame_id = "odom"
    #     marker.header.stamp = rospy.Time.now()
    #     marker.ns = "transformed_points"
    #     marker.id = self.marker_id  # Unique ID for the line strip
    #     marker.type = Marker.LINE_STRIP
    #     marker.action = Marker.ADD
    #     marker.pose.orientation.w = 1.0

    #     # Set the scale of the line
    #     marker.scale.x = 0.1  # 선의 두께

    #     # Set the color
    #     marker.color.r = 1.0
    #     marker.color.g = 0.0
    #     marker.color.b = 0.0
    #     marker.color.a = 1.0

    #     # Add points to the marker
    #     for point in [(-5,5),(15,5)]:
    #         p = Point()
    #         p.x = point[0]
    #         p.y = point[1]
    #         p.z = 0.0
    #         marker.points.append(p)

    #     # Publish the marker
    #     self.marker_pub.publish(marker)
    #     self.marker_id += 1  # Increment the marker ID for the next marker


    def global2local(self, points, x0, y0, theta0):
        local_points = []
        cos_theta = np.cos(theta0)
        sin_theta = np.sin(theta0)

        for point in points:
            x_prime = point[0] - x0
            y_prime = point[1] - y0
            x_local = cos_theta * x_prime + sin_theta * y_prime
            y_local = -sin_theta * x_prime + cos_theta * y_prime
            local_points.append((x_local, y_local))

        return local_points

    def local2global(self, points, x0, y0, theta0):
        global_points = []
        cos_theta = np.cos(-theta0)
        sin_theta = np.sin(-theta0)

        for point in points:
            x_prime = cos_theta * point[0] + sin_theta * point[1]
            y_prime = -sin_theta * point[0] + cos_theta * point[1]
            global_points.append((x0 + x_prime, y0 + y_prime))

        return global_points


    def get_yaw_from_quaternion(self, q):
        """
        Convert a quaternion into yaw angle (in radians)
        """
        import tf.transformations
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2]


if __name__ == "__main__":
    hz = 50
    rospy.init_node("Stanleycontroller")
    node = Stanleycontroller(hz)
    rate = rospy.Rate(hz)   # 50 Hz
    while not rospy.is_shutdown():
        rate.sleep()
