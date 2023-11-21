#!/usr/bin/env python3
import random
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from wr_hsi_sensing.msg import CoordinateMsg

from wr_drive_msgs.msg import DriveTrainCmd
from geometry_msgs.msg import PoseStamped
from finder import offset_lidar_data
from copy import deepcopy

import math

t=100
dist = 7

#coord[x,y]
class Obstacle:
    def __init__(self, coord1, coord2):
        self.coord1 = coord1
        self.coord2 = coord2
    def moveRight(self, x):
        self.coord1[0] = self.coord1[0] + x 
        self.coord2[0] = self.coord2[0] + x 
    def moveUp(self, x):
        self.coord1[1] = self.coord1[1] + x 
        self.coord2[1] = self.coord2[1] + x 
    def getAngle1(self) -> int:
        return int(pointsToAngle(self.coord1[0], self.coord1[1]))
    def getAngle2(self) -> int:
        return int(pointsToAngle(self.coord2[0], self.coord2[1]))
    def setObstacleFront(self):
        self.coord1[0] = 3
        self.coord2[0] = -2

obstacle = Obstacle([3,3],[2,3])
obstacle.moveRight(-2)

def get_laser_ranges(t=0):
    inputData = []

    # angle1 = int(math.asin(height/(cartesianToRadian(xDist1,height))))

    # angle2 = int(math.asin(height/(cartesianToRadian(xDist2,height))))
    angle1 = obstacle.getAngle1()
    angle2 = obstacle.getAngle2()
    

    for x in range(180):
        if(x >= angle1 and x <= angle2):
            if(x != 0):
                modifier = (obstacle.coord1[1]/math.tan(math.radians(x))) - obstacle.coord1[0] # get x distance x = height/tan(angle)
            inputData.append(cartesianToRadian(obstacle.coord1[0]+modifier, obstacle.coord1[1]))

            #rospy.logerr(str(x) + " " +str(cartesianToRadian(obstacle.coord1[0]+modifier, obstacle.coord1[1])))
        else:
            inputData.append(10)

    for i in range(180):
        inputData.append(.3)

    # for x in range(180):
    #     distance = 10
    #     if t -90 <= x and x <= t and x != 0:
    #         distance = dist
    #     inputData.append(distance)
    # for i in range(180):
    #     inputData.append(.3)
    return inputData


def pointsToAngle(x,y) -> float:
    return math.degrees(math.atan2(y,x))

def cartesianToRadian(x,y) -> float:
    return math.sqrt(x*x+y*y)

def run_mock_data() -> None:
    global mock_heading
    global laser_adjust

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
    rospy.Subscriber('/control/drive_system/cmd', DriveTrainCmd, updateHeading)
    rospy.Subscriber('/laser_adjuster', Float64, updateLaserAdjuster)

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
    laser_adjust = 0
    mock_gps = CoordinateMsg()
    mock_gps.latitude = 0
    mock_gps.longitude = 0

    print("sent fake nav data")

    sleeper = rospy.Rate(10)
    while not rospy.is_shutdown():
        laser.ranges = get_laser_ranges(t)
        distanceData.publish(laser)

        zero_msg.header.seq = frameCount
        zero_msg.header.stamp = rospy.get_rostime()
        frameCount += 1

        mock_heading_pub.publish(mock_heading)
        mock_gps_pub.publish(mock_gps)
        zero_pub.publish(zero_msg)
        sleeper.sleep()
def updateLaserAdjuster(data):
    global laser_adjust
    laser_adjust = data.data

def updateHeading(data) -> None:
    global mock_heading
    global t
    global dist
    global height
    mock_heading = (
        mock_heading + (data.left_value - data.right_value)*3) % 360
    #t+= ((data.left_value - data.right_value)*3)*(4)
    obstacle.moveUp(-(data.left_value + data.right_value)*.05)

    obstacle.moveRight(-laser_adjust*.03)
    if(obstacle.coord1[1] < .1 or obstacle.coord2[1] <.1):
        obstacle.moveUp(5)
        obstacle.setObstacleFront()
        rospy.logerr(obstacle.coord1[0])
    


    # dist = dist - .1*((data.left_value + data.right_value)/2)
    # #t+=2
    # t%=360
    # if(dist <= 0.1 or (dist < 2.8 and data.left_value >= .298 and data.right_value >= .298)):
    #     t=100+(random.randint(0,70))
    #     dist=7
   


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
