# rml_bts

# 패키지 설명

1. 각 센서 노드는 센서 데이터를 publish
  - GPS
    - Topic name: /fix
    - Msg type: Sensor_msgs/NavSatFix
  - Odom

## 명령어

```
# 센서 노드 실행
roslaunch nmea_navsat_driver nmea_serial_driver.launch  # GPS
roslaunch scout_bringup scout_mini_robot_base.launch  # Odometry
roslaunch vectornav vectornav.launch  # IMU

# Sensor fusion 노드 실행
roslaunch scout_localization scout_localization.launch

# teleop_keyboard 노드 실행
roslaunch scout_bringup scout_teleop_keyboard.launch
```
