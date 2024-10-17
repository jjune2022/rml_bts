# rml_bts

# 패키지 설명

1. 각 센서 노드는 센서 데이터를 publish
  - GPS
    - Topic name: /fix
    - Msg type: Sensor_msgs/NavSatFix
  - Odom
    - Topic name: /odom
    - Msg type: nav_msgs/Odometry
  - IMU
    - Topic name: /vectornav/IMU
    - Msg type: Sensor_msgs/IMU
2. ekf_localization 노드는 /odom, /vectornav/IMU, /odometry/gps 토픽을 subscribe하고 /odometry/filtered 토픽을 publish
3. navsat_transfoem 노드는 /fix, /vectornav/IMU, /odometry/filtered 토픽을 subscribe하고 /odometry/gps 토픽을 publish
- 즉, 

## 명령어

```
# 센서 노드 실행
roslaunch nmea_navsat_driver nmea_serial_driver.launch  # GPS
roslaunch scout_bringup scout_mini_robot_base.launch    # Odometry
roslaunch vectornav vectornav.launch                    # IMU

# Sensor fusion 노드 실행
roslaunch scout_localization scout_localization.launch

# teleop_keyboard 노드 실행
roslaunch scout_bringup scout_teleop_keyboard.launch
```
