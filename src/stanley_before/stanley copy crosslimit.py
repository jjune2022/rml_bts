import os
import sys
import rospy
import numpy as np
import tf2_ros
import matplotlib.pyplot as plt
from math import cos, sin
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

class Stanleycontroller:
    def __init__(self, hz=50, k=1):
        rospy.init_node('Stanleycontroller')
        rospy.Subscriber("/odom", Odometry, self.odom_update)
        rospy.Subscriber('/way_points', Path, self.waypoints_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.local_points = []
        self.global_points = None
        self.hz = hz
        self.dt = 1 / hz  # time step
        self.odom_pose = None  # position x, y, z, orientation x, y, z, w
        self.odom_twist = None  # linear x, y, z, angular x, y, z
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta0 = 0.0
        self.k = k            # Stanley controller gain for angle correction
        self.min_vel = 0.0
        self.marker_id = 0
        self.max_vel = 5
        self.angles = np.linspace(0, np.pi, 100)
        self.waypoints = []
        self.velocity = 1  # m/s

    # odom은 bringup으로부터 자동으로 publish됨(global좌표)
    def odom_update(self, data):
        self.odom_pose = data.pose.pose
        self.odom_twist = data.twist.twist
        self.theta0 = self.get_yaw_from_quaternion(self.odom_pose.orientation)
        # front wheel center
        self.x0, self.y0 = self.odom_pose.position.x + 0.175 * cos(self.theta0), self.odom_pose.position.y + 0.175 * sin(self.theta0)

        # x_data, y_data에 현재 위치 추가
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
            dist = np.sqrt((self.x0 - wx) ** 2 + (self.y0 - wy) ** 2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Waypoint to track is the closest one or next waypoint
        target_wp = self.waypoints[closest_idx]

        # Compute the heading error
        path_angle = np.arctan2(target_wp[1] - self.y0, target_wp[0] - self.x0)  # rad
        cross_track_error = np.sqrt((target_wp[0] - self.x0) ** 2 + (target_wp[1] - self.y0) ** 2)
        heading_error = path_angle - self.theta0

        #Normalize heading error to [-pi, pi]
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

        # Stanley control
        angle_correction = heading_error + np.arctan2(self.k * cross_track_error, v0)
        omega = angle_correction / self.dt  # /ref_pos 50초마다 publish한다고 가정
        omega = np.clip(omega, -2.5235, 2.5235)  # Limit angular velocity

        if cross_track_error < 0.04:
            omega = 0

        # Publish control command
        control_cmd = Twist()
        control_cmd.linear.x = velocity
        control_cmd.angular.z = omega
        self.pub.publish(control_cmd)

        # Debug info
        rospy.loginfo(f"Velocity: {velocity}, Angular Velocity: {omega}, Heading Error: {heading_error}, Path Distance: {cross_track_error}, Target: {target_wp}")

        # heading error와 path distance를 리스트에 추가
        heading_error_data.append(heading_error)
        cross_track_error_data.append(cross_track_error)

        # 추가: heading error와 path distance의 x축 값을 odom.pose.x 값으로 저장
        odom_x_for_heading_error.append(self.x0)
        odom_x_for_cross_track_error.append(self.x0)

        # 종료 조건: odom의 x 값이 path의 x 값에 도달한 경우
        if abs(self.x0 - path_x_data[-1]) < 0.01:  # 작은 오차를 허용
            rospy.loginfo("Reached the target x position. Shutting down the node.")
            save_results_and_shutdown()

    def get_yaw_from_quaternion(self, q):
        """
        Convert a quaternion into yaw angle (in radians)
        """
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2]

def save_graph():
    # 디렉터리가 없으면 생성
    save_dir = os.path.expanduser('~/scout_sim/odom')
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # 고유 파일 이름 생성
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f'combined_graph_{timestamp}.png'
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
    save_dir = os.path.expanduser('~/scout_sim/table')
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # 고유 파일 이름 생성
    timestamp = datetime.now().strftime('%Y%m%d_%H%M')

    # 이미지 파일 이름 생성
    img_filename = f'rmse_table_{timestamp}.png'
    img_filepath = os.path.join(save_dir, img_filename)

    # 표를 이미지로 저장
    fig, ax = plt.subplots(figsize=(6, 2))  # Adjust size as needed
    ax.axis('off')
    table = ax.table(cellText=df.values, colLabels=df.columns, cellLoc='center', loc='center')
    table.scale(1, 2)  # Adjust scaling to make the table fit well
    plt.savefig(img_filepath, bbox_inches='tight')
    plt.close(fig)

    print(f"RMSE table saved as {img_filepath}")

def save_results_and_shutdown():
    # 그래프 저장
    save_graph()

    # RMSE 계산
    cross_track_rmse = round(calculate_rmse(cross_track_error_data),3)
    heading_rmse = round(calculate_rmse(heading_error_data),3)

    # RMSE 표 저장
    save_rmse_table(cross_track_rmse, heading_rmse)

    # ROS 노드 종료
    rospy.signal_shutdown("Results saved, shutting down.")

def signal_handler(sig, frame):
    save_results_and_shutdown()

if __name__ == "__main__":
    hz = 50
    rospy.init_node("Stanleycontroller")
    node = Stanleycontroller(hz)

    # SIGINT (Ctrl+C) 시그널 핸들러 설정
    signal.signal(signal.SIGINT, signal_handler)

    rate = rospy.Rate(hz)   # 50 Hz
    while not rospy.is_shutdown():
        rate.sleep()

    # 종료 시 결과 저장
    save_results_and_shutdown()
