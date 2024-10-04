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

# x, y 위치 데이터를 저장할 리스트 초기화
x_data = []
y_data = []

# heading error와 path distance 데이터를 저장할 리스트 초기화
heading_error_data = []
cross_track_error_data = []

# 추가: heading error와 path distance에 해당하는 x축 값 (odom.pose.x)을 저장할 리스트 초기화
odom_x_for_heading_error = []
odom_x_for_cross_track_error = []

# 경로 데이터 저장
path_x_data = []
path_y_data = []

pure_pursuit_ld_gain = []
pure_pursuit_vel = []

class PurePursuitController:
    def __init__(self, hz=50, ld_gain=1.0, ld_min = 0.6, car_wheel_base=0.463):
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
        self.velocity = 0.5   # m/s
        pure_pursuit_ld_gain.append(self.ld_gain)
        pure_pursuit_vel.append(self.velocity)
        self.waypoints = []

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
        self.run_pure_pursuit()

    def run_pure_pursuit(self):
        if len(self.waypoints) < 2 or self.odom_pose is None:
            return

        v0 = self.velocity
        ld = self.ld_gain * self.velocity+self.ld_min  # Lookahead distance
        closest_idx = 0
        min_dist = float('inf')

        # Find the closest waypoint ahead of the robot
        for i in range(len(self.waypoints)):
            dist = sqrt((self.x0 - self.waypoints[i][0]) ** 2 + (self.y0 - self.waypoints[i][1]) ** 2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        cross_track_error_data.append(min_dist)
        # Compute the heading error
        path_angle = np.arctan2(self.waypoints[closest_idx+1][1] - self.waypoints[closest_idx][1],
                                 self.waypoints[closest_idx+1][0] - self.waypoints[closest_idx][0])  # rad
        heading_error =path_angle - self.theta0
        #Normalize heading error to [-pi, pi]
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
        heading_error_data.append(heading_error)
        odom_x_for_heading_error.append(self.x0)
        odom_x_for_cross_track_error.append(self.x0)


        # Find the lookahead point
        # Find the closest waypoint ahead of the robot
        for i in range(len(self.waypoints)):
            dist = sqrt((self.x0 - 0.232 * cos(self.theta0)- self.waypoints[i][0]) ** 2 + (self.y0 - 0.232 * sin(self.theta0)- self.waypoints[i][1]) ** 2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        lookahead_point = None
        for i in range(closest_idx, len(self.waypoints)):
            dist = sqrt((((self.x0 - 0.232 * cos(self.theta0) )) - self.waypoints[i][0]) ** 2 + (((self.y0- 0.232 * sin(self.theta0))) - self.waypoints[i][1]) ** 2)
            if abs(dist -ld) < 0.1 and self.x0 < self.waypoints[i][0]:
                lookahead_point = self.waypoints[i]
                break

        if lookahead_point is None:
            lookahead_point = self.waypoints[-1]  # Fallback to the last point

        # Publish the lookahead point as a Marker
        self.publish_lookahead_marker(lookahead_point)

        # Calculate the steering angle
        alpha = atan2(lookahead_point[1] - (self.y0- 0.232 * sin(self.theta0)), lookahead_point[0] - ((self.x0 - 0.232 * cos(self.theta0) ))) - self.theta0
        steering_angle = atan2(2 * self.car_wheel_base * sin(alpha), ld)
        omega = steering_angle / self.dt
        omega = np.clip(omega, -2.5235, 2.5235)  # Limit angular velocity

        # Publish control command
        control_cmd = Twist()
        control_cmd.linear.x = self.velocity
        control_cmd.angular.z = omega
        self.pub.publish(control_cmd)

        rospy.loginfo(f"Velocity: {self.velocity}, Steering Angle: {steering_angle}, Lookahead Point: {lookahead_point}")

        # 종료 조건: odom의 x 값이 path의 x 값에 도달한 경우
        if abs(self.x0 - path_x_data[-1]) < 0.5 and abs(self.y0 - path_y_data[-1]) < 0.5:
            rospy.loginfo("Reached the target position. Shutting down the node.")
            save_results_and_shutdown()

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

    def get_yaw_from_quaternion(self, q):
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2]

def save_graph():
    # 디렉터리가 없으면 생성
    save_dir = os.path.expanduser(f'~/scout_sim/odom/pure_v{pure_pursuit_vel}_tuning')
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # 고유 파일 이름 생성
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f'pure_v{pure_pursuit_vel}_k{pure_pursuit_ld_gain}.png'
    filepath = os.path.join(save_dir, filename)

    fig, axs = plt.subplots(3, 1, figsize=(10, 15))  # 3개의 서브플롯을 세로로 배치

    # 첫 번째 서브플롯: heading error 그래프
    axs[0].plot(odom_x_for_heading_error, heading_error_data, color='green')
    axs[0].set_title('Heading Error')
    axs[0].set_xlabel('X Position')
    axs[0].set_ylabel('Heading Error (radians)')

    # 두 번째 서브플롯: path distance 그래프
    axs[1].plot(odom_x_for_cross_track_error, cross_track_error_data, color='orange')
    axs[1].set_title('Cross-track Error')
    axs[1].set_xlabel('X Position')
    axs[1].set_ylabel('Distance (meters)')

    # 세 번째 서브플롯: 로봇 경로 및 계획된 경로 그래프
    axs[2].plot(x_data, y_data, label='Scout mini', color='blue')
    if path_x_data and path_y_data:
        axs[2].plot(path_x_data, path_y_data, label='Path', color='red', linestyle='--')
    axs[2].set_xlabel('X Position')
    axs[2].set_ylabel('Y Position')
    axs[2].set_title('Odometry Position and Planned Path')
    axs[2].legend()

    # 서브플롯들 사이의 간격 조정
    plt.tight_layout()

    # 그래프 저장
    plt.savefig(filepath)
    plt.close(fig)
    print(f"Graph saved as {filepath}")

def calculate_rmse(errors):
    """Calculate the Root Mean Square Error (RMSE)"""
    return np.sqrt(np.mean(np.square(errors)))

def save_rmse_table(cross_track_rmse, heading_rmse):
    # 데이터 프레임 생성
    data = {
        'Metric': ['Cross Track Error RMSE (m)', 'Heading Error RMSE (rad)'],
        'Value': [cross_track_rmse, heading_rmse]
    }
    df = pd.DataFrame(data)

    # 디렉터리가 없으면 생성
    save_dir = os.path.expanduser(f'~/scout_sim/table/pure_v{pure_pursuit_vel}_tuning')
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # 고유 파일 이름 생성
    timestamp = datetime.now().strftime('%Y%m%d_%H%M')

    # 이미지 파일 이름 생성
    img_filename = f'pure_v{pure_pursuit_vel}_{pure_pursuit_ld_gain}.png'
    img_filepath = os.path.join(save_dir, img_filename)

    # 표를 이미지로 저장
    fig, ax = plt.subplots(figsize=(6, 2))  # Adjust size as needed
    ax.axis('off')
    table = ax.table(cellText=df.values, colLabels=df.columns, cellLoc='center', loc='center')
    table.scale(1, 2)  # Adjust scaling to make the table fit well
    plt.savefig(img_filepath, bbox_inches='tight')
    plt.close(fig)

    print(f"RMSE table saved as {img_filepath}")

def save_data_to_txt():
    save_dir = os.path.expanduser(f'~/scout_sim/odom/pure_v{pure_pursuit_vel}_tuning')
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # x, y 데이터 저장
    np.savetxt(os.path.join(save_dir, 'purepursuit_xy_data.txt'), np.column_stack((x_data, y_data)), header="x y", comments='')

    # heading error, cross track error 데이터 저장
    np.savetxt(os.path.join(save_dir, 'purepursuit_errors.txt'), np.column_stack((odom_x_for_heading_error, heading_error_data, cross_track_error_data)), header="x heading_error cross_track_error", comments='')

    print(f"Data saved as text files in {save_dir}")

# save_results_and_shutdown 함수 내에서 텍스트 데이터 저장 호출
def save_results_and_shutdown():
    # 그래프 저장
    save_graph()

    # 텍스트 데이터 저장
    save_data_to_txt()

    # RMSE 계산
    cross_track_rmse = round(calculate_rmse(cross_track_error_data), 3)
    heading_rmse = round(calculate_rmse(heading_error_data), 3)
    # RMSE 표 저장
    save_rmse_table(cross_track_rmse, heading_rmse)
    # RMSE print
    print()
    print()
    print(f'v{pure_pursuit_vel}, ld_gain{pure_pursuit_ld_gain} | cross_track: {cross_track_rmse}, heading: {heading_rmse}')
    print()
    print()
    # ROS 노드 종료
    rospy.signal_shutdown("Results saved, shutting down.")
def signal_handler(sig, frame):
    save_results_and_shutdown()

if __name__ == "__main__":
    hz = 50
    rospy.init_node("PurePursuitController")
    node = PurePursuitController(hz)

    signal.signal(signal.SIGINT, signal_handler)

    rate = rospy.Rate(hz)   # 50 Hz
    while not rospy.is_shutdown():
        rate.sleep()

    save_results_and_shutdown()
