#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan 
from wr_logic_ai.msg import NavigationMsg

from wr_drive_msgs.msg import DriveTrainCmd
from geometry_msgs.msg import PoseStamped
from finder import offset_lidar_data

import math

def updateHeading(data) -> None:
    global nav
    nav.heading = (nav.heading + (data.right_value - data.left_value)*10) % 360

def get_laser_ranges(t = 0):
        inputData = []
        for x in range(360):
            distance = 10
            if t <= x and x <= t + 40:
                distance = 3
            inputData.append(distance)
        return inputData

def run_mock_data() -> None:
    rospy.init_node('publish_fake_data', anonymous=False)
    distanceData = rospy.Publisher('/scan', LaserScan, queue_size=10)
    navigation = rospy.Publisher('/nav_data', NavigationMsg, queue_size=10)
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

    #Testing publishers and subscribers
    rospy.Subscriber('/control/drive_system/cmd', DriveTrainCmd, updateHeading)

    laser = LaserScan()
    #vara.intensities

    inputData = []
    #laser.intensities()

    #laser.angle_min = 0.
    laser.angle_max = 2 * math.pi
    laser.angle_increment = math.pi / 180
    laser.time_increment = 0
    laser.scan_time = 1
    laser.range_min = 0
    laser.range_max = 150
    laser.ranges = get_laser_ranges(0)
    laser.header.frame_id = "map"
    laser.intensities = []

    nav = NavigationMsg()

    nav.cur_lat = 0
    nav.cur_long = 0
    nav.heading = 0

    nav.tar_lat = 10
    nav.tar_long = 0

    print("sent fake nav data")

    sleeper = rospy.Rate(10)
    t = 0
    while not rospy.is_shutdown():
        laser.ranges = get_laser_ranges(t)
        distanceData.publish(laser)
    
        zero_msg.header.seq = frameCount
        zero_msg.header.stamp = rospy.get_rostime()
        frameCount += 1

        navigation.publish(nav) #send nav data to subscriber in obstacle avoidance
        zero_pub.publish(zero_msg)
        sleeper.sleep()
        t += 2
        t %= 360
    
def display_data(data) -> None:
    data.ranges = offset_lidar_data(data.ranges, math.degrees(data.angle_increment), True)
    scan_rviz_pub = rospy.Publisher('/scan_rviz', LaserScan, queue_size=10)
    scan_rviz_pub.publish(data)

def run_real_data() -> None:
    rospy.Subscriber('/scan', LaserScan, display_data)
    
if __name__ == '__main__':
    #Run fake data
    run_mock_data()

    #Run lidar data
    #run_real_data()
