#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from wr_hsi_sensing.msg import CoordinateMsg

from wr_drive_msgs.msg import DriveTrainCmd
from geometry_msgs.msg import PoseStamped
from finder import offset_lidar_data
from copy import deepcopy

import math


def get_laser_ranges(t=0):
    inputData = []
    for x in range(360):
        distance = 10
        if t <= x and x <= t + 40:
            distance = 0.5
        inputData.append(distance)
    return inputData


def run_mock_data() -> None:
    global mock_heading

    distanceData = rospy.Publisher('/scan', LaserScan, queue_size=10)
    mock_heading_pub = rospy.Publisher('/heading_data', Float64, queue_size=10)
    mock_gps_pub = rospy.Publisher(
        '/gps_coord_data', CoordinateMsg, queue_size=10)
    zero_pub = rospy.Publisher('/debug_zero', PoseStamped, queue_size=10)
    zero_msg = PoseStamped()
    zero_msg.pose.position.x = 0
    zero_msg.pose.position.y = 0
    zero_msg.pose.position.z = 0
    zero_msg.pose.orientation.x = 0
    zero_msg.pose.orientation.y = 0
    zero_msg.pose.orientation.z = 0
    zero_msg.pose.orientation.w = 1
    zero_msg.header.frame_id = "laser"
    frameCount = 0

    # Testing publishers and subscribers
    #rospy.Subscriber('/control/drive_system/cmd', DriveTrainCmd, updateHeading)

    laser = LaserScan()
    # vara.intensities

    #laser.angle_min = 0.
    laser.angle_max = 2 * math.pi
    laser.angle_increment = math.pi / 180
    laser.time_increment = 0
    laser.scan_time = 1
    laser.range_min = 0
    laser.range_max = 150
    laser.ranges = get_laser_ranges(0)
    laser.header.frame_id = "laser"
    laser.intensities = []

    mock_heading = 0
    mock_gps = CoordinateMsg()
    mock_gps.latitude = 39.397283363899916
    mock_gps.longitude = -113.12379807045856

    print("sent fake nav data")

    sleeper = rospy.Rate(10)
    t = 0
    while not rospy.is_shutdown():
        laser.ranges = get_laser_ranges(t)
        distanceData.publish(laser)

        zero_msg.header.seq = frameCount
        zero_msg.header.stamp = rospy.get_rostime()
        frameCount += 1

        #mock_heading_pub.publish(mock_heading)
        mock_gps_pub.publish(mock_gps)
        zero_pub.publish(zero_msg)
        sleeper.sleep()
        t += 2
        t %= 360


def updateHeading(data) -> None:
    global mock_heading
    mock_heading = (
        mock_heading + (data.right_value - data.left_value)*10) % 360


def display_data(data) -> None:
    rviz_data = deepcopy(data)
    rviz_data.ranges = offset_lidar_data(
        rviz_data.ranges, math.degrees(rviz_data.angle_increment), True)
    scan_rviz_pub = rospy.Publisher('/scan_rviz', LaserScan, queue_size=10)
    scan_rviz_pub.publish(rviz_data)


def run_real_data() -> None:
    rospy.Subscriber('/scan', LaserScan, display_data)


if __name__ == '__main__':
    rospy.init_node('publish_fake_data', anonymous=False)

    if rospy.get_param('~run_in_mock', True):
        # Run fake data
        run_mock_data()
    else:
        # Run lidar data
        sub = rospy.Subscriber('/scan', LaserScan, display_data)
        # run_real_data()
        rospy.spin()
