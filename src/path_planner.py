import rospy
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
import numpy as np


class Pathplanner:
    def __init__(self):
        rospy.init_node('Pathplanner')
        self.marker_pub = rospy.Publisher('/transformed_points_marker', Marker, queue_size=10)
        self.path_pub = rospy.Publisher('/way_points', Path, queue_size=10)
        self.marker_id = 0  # Initialize marker ID


        # #line
        # self.waypoints = [( 0.1 * i, 1) for i in range(500)]
        # # sine
        self.waypoints = self.generate_sine_wave_waypoints()

        # # Generate infinity loop waypoints
        # self.waypoints = self.generate_infinity_loop_waypoints()

    def generate_sine_wave_waypoints(self):
        """
        Generate waypoints along a sine wave.
        """
        # Parameters for sine wave
        amplitude = 1.5  # Amplitude of the sine wave
        frequency = 1  # Frequency of the sine wave
        phase = 0         # Phase shift of the sine wave
        x_start = 0      # Start x value
        x_end = 50       # End x value
        num_points = 1000 # Number of points

        x_values = np.linspace(x_start, x_end, num_points)
        y_values = amplitude * np.sin(frequency * x_values + phase)

        waypoints = list(zip(x_values, y_values))
        return waypoints

    def generate_infinity_loop_waypoints(self):
        """
        Generate waypoints along an infinity loop (lemniscate of Gerono).
        """
        num_points = 1000  # Number of points
        a = 50  # Scaling factor

        t_values = np.linspace(0, 2 * np.pi, num_points)
        x_values = a * np.sin(t_values)
        y_values = a * np.sin(t_values) * np.cos(t_values)

        waypoints = list(zip(x_values, y_values))
        return waypoints


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
        for x, y in self.waypoints:
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
        for x, y in self.waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "odom"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0  # No rotation
            path.poses.append(pose_stamped)

        # Publish the path
        self.path_pub.publish(path)


if __name__ == '__main__':
    path_planner = Pathplanner()
    rate = rospy.Rate(50)  # 50 Hz
    while not rospy.is_shutdown():
        path_planner.publish_way_points_marker()
        path_planner.publish_way_points_path()
        rate.sleep()
