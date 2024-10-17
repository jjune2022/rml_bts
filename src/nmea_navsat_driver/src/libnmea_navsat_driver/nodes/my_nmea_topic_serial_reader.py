#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Defines the main method for the nmea_topic_serial_reader executable."""

import serial
import pynmea2

from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix
import rospy

from libnmea_navsat_driver.driver import RosNMEADriver


def main():
    """Create and run the nmea_topic_serial_reader ROS node.

    Opens a serial device and publishes data from the device as nmea_msgs.msg.Sentence messages.

    :ROS Parameters:
        - ~port (str)
            Path of the serial device to open.
        - ~baud (int)
            Baud rate to configure the serial device.

    :ROS Publishers:
        - nmea_sentence (nmea_msgs.msg.Sentence)
            Publishes each line from the openserial device as a new message. The header's stamp is
            set to the rostime when the data is read from the serial device.
    """
    rospy.init_node('my_nmea_topic_serial_reader')

    nmea_pub = rospy.Publisher("nmea_sentence", Sentence, queue_size=1)
    gps_pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=1)

    serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baud', 115200)

    # Get the frame_id
    frame_id = 'map'

    try:
        GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)
        while not rospy.is_shutdown():
            data = GPS.readline().strip()

            # sentence = Sentence()
            # sentence.header.stamp = rospy.get_rostime()
            # sentence.header.frame_id = frame_id

            try:
                # sentence.sentence = data.decode('ascii')
                decoded_data = data.decode('ascii')
                msg = pynmea2.parse(decoded_data)

                if isinstance(msg, pynmea2.RMC):
                    # NavSatFix 메시지 생성
                    navsat_msg = NavSatFix()
                    navsat_msg.header.stamp = rospy.get_rostime()
                    navsat_msg.header.frame_id = frame_id
                    navsat_msg.latitude = msg.latitude
                    navsat_msg.longitude = msg.longitude
                    navsat_msg.altitude = 0.0

                    # GPS 데이터 publish
                    gps_pub.publish(navsat_msg)

                    print(navsat_msg.header)
                    print("latitude: ", navsat_msg.latitude)
                    print("longitude: ", navsat_msg.longitude, '\n')

            except UnicodeError as e:
                rospy.logwarn("Skipped reading a line from the serial device because it could not be "
                              "decoded as an ASCII string. The bytes were {0}".format(data))
            # else:
            #     nmea_pub.publish(sentence)

    except rospy.ROSInterruptException:
        GPS.close()  # Close GPS serial port
