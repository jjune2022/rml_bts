import rospy
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
import numpy as np
import math
import sys
sys.path.append('/home/rml/scout_sim/src/erp-cml/path/')
from CubicSpline import cubic_spline_planner

dl =0.1 # distance of interpolateed points

class Pathplanner:
    def __init__(self):
        rospy.init_node('Pathplanner')
        self.marker_pub = rospy.Publisher('/transformed_points_marker', Marker, queue_size=10)
        self.path_pub = rospy.Publisher('/way_points', Path, queue_size=10)
        self.marker_id = 0  # Initialize marker ID


        ### generate path for 4cases , ck: curvature
        self.cx, self.cy, self.cyaw, self.ck = get_straight_course(dl)
        #self.cx, self.cy, self.cyaw, self.ck = get_straight_course1(dl)
        #self.cx, self.cy, self.cyaw, self.ck = get_forward_course(dl)
        #self.cx, self.cy, self.cyaw, self.ck = get_switch_back_course(dl)

    def publish_way_points_marker(self):
        """
        Publish the waypoints to RViz as a Marker
        """
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "transformed_points"
        marker.id = self.marker_id  # Unique ID for the line strip
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Set the scale of the line
        marker.scale.x = 0.1  # Line thickness

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Add points to the marker
        for x, y in zip(self.cx, self.cy):
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        # Publish the marker
        self.marker_pub.publish(marker)
        self.marker_id += 1  # Increment the marker ID for the next marker

    def publish_way_points_path(self):
        """
        Publish the waypoints to RViz as a Path
        """
        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = rospy.Time.now()

        # Add points to the path
        for x, y , yaw in zip(self.cx, self.cy, self.cyaw):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "odom"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = yaw
            path.poses.append(pose_stamped)

        # Publish the path
        self.path_pub.publish(path)
def get_straight_course(dl):
    ax = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_straight_course1(dl):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    cyaw = [i - math.pi for i in cyaw]

    return cx, cy, cyaw, ck


def get_forward_course(dl):
    ax = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
    ay = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_switch_back_course(dl):
    ax = [0.0, 30.0, 6.0, 20.0, 35.0]
    ay = [0.0, 0.0, 20.0, 35.0, 20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    ax = [35.0, 10.0, 0.0, 0.0]
    ay = [20.0, 30.0, 5.0, 0.0]
    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    cyaw2 = [i - math.pi for i in cyaw2]
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)

    return cx, cy, cyaw, ck


if __name__ == '__main__':
    path_planner = Pathplanner()
    rate = rospy.Rate(50)  # 50 Hz
    while not rospy.is_shutdown():
        path_planner.publish_way_points_marker()
        path_planner.publish_way_points_path()
        rate.sleep()
