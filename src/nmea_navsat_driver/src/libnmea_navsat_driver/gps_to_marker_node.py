#! /usr/bin/env python3

import rospy

from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import utm

def cb_gps_to_marker(data):
    ############## 좌표 변환 ###############
    utm_coord = utm.from_latlon(data.latitude, data.longitude)

    point = Point()
    # point.x = utm_coord[0]
    # point.y = utm_coord[1]
    
    point.x = utm_coord[0] - 517217.18
    point.y = utm_coord[1] - 3936700.37
    point.z = 0.0

    print("utm_x: ", point.x)
    print("utm_y: ", point.y)
    #######################################
    
    # Marker 메시지 생성
    marker = Marker()
    marker.header.stamp = rospy.get_rostime()
    marker.header.frame_id = "local_map"
    
    marker.ns = "gps_points"
    marker.type = Marker.POINTS

    marker.points.append(point)
    
    # marker.pose.position.x = data.latitude
    # marker.pose.position.y = data.longitude
    # marker.pose.position.z = data.altitude

    # marker.pose.orientation.x = 0.0
    # marker.pose.orientation.y = 0.0
    # marker.pose.orientation.z = 0.0
    # marker.pose.orientation.w = 1.0
    
    marker.scale.x = 2.0
    marker.scale.y = 2.0
    # marker.scale.z = 2.0
    
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    
    # Marker 데이터 publish
    pub.publish(marker)

if __name__ == "__main__":
    rospy.init_node("gps_to_marker_node")
    
    rospy.Subscriber("/gps_data", NavSatFix, cb_gps_to_marker)

    pub = rospy.Publisher("/gps_to_marker", Marker, queue_size=1)

    print("OK")

    rospy.spin()
