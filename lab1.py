#! /usr/bin/python3

import rospy
import math

from sensor_msgs.msg import LaserScan

pub_old = rospy.Publisher('/scan_laser_before', LaserScan, queue_size = 10)
pub_new = rospy.Publisher('/scan_laser_after', LaserScan, queue_size = 10)

def callback(msg):
    pub_old.publish(msg)
    msg.ranges = get_filtered_data(msg.ranges, msg.angle_min, msg.angle_increment)
    pub_new.publish(msg)

def get_coordinates(r, theta):
    return r * math.cos(theta), r * math.sin(theta)

def get_angle(angle_min, angle_increment, position):
    return angle_min + angle_increment * position

def get_distance(a, b):
    return math.sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2))

def get_filtered_data(data, angle_min, angle_increment):
    filtered_data = []
    step = 3
    max_distance = 0.3
    max_filter_value = 9000.0

    for i in range(step):
        filtered_data.append(max_filter_value)

    for i in range(step, len(data) - step):
        prev_point = get_coordinates(data[i - step], get_angle(angle_min, angle_increment, i - step))
        current_point = get_coordinates(data[i], get_angle(angle_min, angle_increment, i))
        next_point = get_coordinates(data[i + step], get_angle(angle_min, angle_increment, i + step))

        if get_distance(prev_point, current_point) > max_distance or get_distance(next_point, current_point) > max_distance:
            filtered_data.append(max_filter_value)
        else:
            filtered_data.append(data[i])

    for i in range(step):
        filtered_data.append(max_filter_value)

    return filtered_data

rospy.init_node('lab1_scan_laser')
rospy.Subscriber('base_scan', LaserScan, callback)
r = rospy.Rate(0.5)

while not (rospy.is_shutdown()):
    r.sleep()