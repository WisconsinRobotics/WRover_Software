from typing import List
import math
import scipy.ndimage as gaussian_smooth
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
from copy import deepcopy
import os
import pdb
import pickle

ROVER_WIDTH = 0.5

scan_rviz_pub = rospy.Publisher("/scan_rviz", LaserScan, queue_size=10)
# TODO (@bennowotny ) This should be disable-able for bandwidth
window_pub = rospy.Publisher("/lidar_windows", PoseArray, queue_size=1)


def calculate_anti_window(d: float) -> int:
    return math.degrees(math.atan((ROVER_WIDTH / 2) / d))

# Returns angular distance (in degrees) from sector parameter to target
# Returns 0 if sector parameter contains target
# TODO: Is there a cleaner way to write the logic?


def get_target_distance(sector_i: int, sector_f: int, target: int, sector_angle: float) -> float:
    # Compute if the target is under the starting sector
    if target < sector_i:
        # Return angular distance between target and starting sector
        return (sector_i - target) * sector_angle
    # Compute if the target is above the last sector
    if target > sector_f:
        # Return the angular distance between the target and last sector
        return (target - sector_f) * sector_angle
    # Otherwise, the target is within the sector bounds
    # Angular distance is then 0
    return 0

# Returns whether or not a valley is as wide or wider than the recommended max valley size


def is_wide_valley(sector_i: int, sector_f: int, max_valley: int) -> bool:
    # TODO: Cleaner way to write this?
    return 1 + sector_f - sector_i > max_valley

# Transforms the raw lidar data from compass coordinates (0 at north, clockwise) to math coordinates (0 at east, counterclockwise)


def offset_lidar_data(data, sector_angle, is_rviz=False):
    offset_data = [0] * len(data)
    if is_rviz:
        for i in range(len(data)):
            offset_data[(i + math.floor(90 / sector_angle) +
                         math.floor(len(data) / 2)) % len(data)] = data[i]
    else:
        for i in range(len(data)):
            offset_data[(i + math.floor(90 / sector_angle)) %
                        len(data)] = data[i]
    return offset_data

# represent a valley as an ordered pair as in (start sector, end sector)
# iterate through sector array to add valleys to new candidate valleys list
# from candidate valleys choose the one that is closest to the target sector
# valleys of any length should be valid since they can only come from smoothing function of adjacent areas
# and correct thresholding

# Get the 'best' valley from the LIDAR data.  The valley is formatted as [Start Sector, End Sector]


def get_valley(
        target: int,
        sector_angle: float,
        threshold: float,
        data: LaserScan,
        smoothing: float = 3) -> List[int]:
    global prevData

    rviz_data = deepcopy(data)
    #rviz_data.ranges = gaussian_smooth.gaussian_filter1d(rviz_data.ranges, smoothing)
    rviz_data.ranges = offset_lidar_data(
        rviz_data.ranges, sector_angle, is_rviz=True)
    scan_rviz_pub.publish(rviz_data)

    # rospy.loginfo(f"{data.ranges}")
    # TODO: remove dependency on this variable by making the mock script more like real hardware input
    if rospy.get_param("~wrover_hw") == "REAL":
        #hist = offset_lidar_data(gaussian_smooth.gaussian_filter1d(data.ranges, smoothing), sector_angle)
        hist = offset_lidar_data(data.ranges, sector_angle)
    # For testing:
    else:
        #hist = gaussian_smooth.gaussian_filter1d(data.ranges, smoothing)
        hist = list(data.ranges)

    # This is to prevent expanding constant obstacles from behind the robot, which can
    # inadvertently and unpredictably (due to sensor noise) block out most of the view
    # frame due to obstacle expansion
    del hist[int(len(hist)/2):]

    # Write the sectors data to an output file for logging
    output_file = open('sectors.data', 'wb')
    pickle.dump(hist, output_file)
    output_file.close()

    # TODO : The names here are a little unclear, maybe some comments/renames (F2)?
    # This chunk code will find obstacles, expand the edges of the obstacle for the length of half the robot (with some extra
    # for redundecy), and make a list of all the obstacles.
    # format: [[left bound index , right bound index], ...]
    obstacle_list = list()
    # this will be an array of two values with the beginning and ending index of the obstacle.
    one_obstacle = []
    for i in range(len(hist)):
        if(hist[i] < threshold):
            one_obstacle.append(i)
        # This prevents single noisy points from blocking out large portions of the drive window
        # TODO (@bennowotny ): This 'obstacle too small' magic number should be a named constant
        elif (len(one_obstacle) > 1):
            left_bound = len(hist)
            right_bound = 0
            for i in range(len(one_obstacle)):
                # Calculate size of anti-window and add to obstacle bounds
                # pass in distance to target to caculate angle that allows robot to pass through
                angleToIncrease = calculate_anti_window(
                    hist[one_obstacle[i]]) / sector_angle

                # Update left and right bound
                left_bound = max(
                    min(left_bound, one_obstacle[i]-angleToIncrease), 0)
                right_bound = min(
                    max(right_bound, one_obstacle[i]+angleToIncrease), len(hist))

            # Check to see if the obstacle we just found can actually be merged with a previous obstacle
            while len(obstacle_list) > 0 and obstacle_list[-1][1] >= left_bound:
                left_bound = min(left_bound, obstacle_list[-1][0])
                right_bound = max(right_bound, obstacle_list[-1][1])
                del obstacle_list[-1]
            obstacle_list.append([left_bound, right_bound])
            one_obstacle.clear()

    # TODO (@bennowotny ): This code is the same as what's in the loop, so it should be abstracted out to its own function
    if (len(one_obstacle) != 0):
        left_bound = len(hist)
        right_bound = 0
        for i in range(len(one_obstacle)):
            # Calculate size of anti-window and add to obstacle bounds
            # pass in distance to target to caculate angle that allows robot to pass through
            angleToIncrease = calculate_anti_window(
                hist[one_obstacle[i]]) / sector_angle

            # Update left and right bound
            left_bound = max(
                min(left_bound, one_obstacle[i]-angleToIncrease), 0)
            right_bound = min(
                max(right_bound, one_obstacle[i]+angleToIncrease), len(hist))

        # Check to see if the obstacle we just found can actually be merged with a previous obstacle
        while len(obstacle_list) > 0 and obstacle_list[-1][1] >= left_bound:
            left_bound = min(left_bound, obstacle_list[-1][0])
            right_bound = max(right_bound, obstacle_list[-1][1])
            del obstacle_list[-1]
        obstacle_list.append([left_bound, right_bound])
        one_obstacle.clear()

    # At this point we make an inverse list of the obstacles to have a 2d list of all
    # the places that we can drive through (our windows)
    window_list = list()
    # If the obstacle list is empty, the window is the entire 360 degree range
    if len(obstacle_list) == 0:
        window_list.append([0, len(hist)])
    else:
        # If obstacle_list does not start on the left bound of lidar
        if obstacle_list[0][0] > 0:
            window_list.append([0, obstacle_list[0][0]])

        for i in range(len(obstacle_list) - 1):
            window_list.append([obstacle_list[i][1], obstacle_list[i + 1][0]])

        # If obstacle_list does not end on the right bound of lidar
        if obstacle_list[len(obstacle_list) - 1][1] < len(hist):
            window_list.append(
                [obstacle_list[len(obstacle_list) - 1][1], len(hist)])

    # print("obstacle list:")
    # print(obstacle_list)

    # TODO (Ben Nowotny) Remove, probably a little laggy
    window_msg = PoseArray()
    window_msg.header.frame_id = "laser"
    for window in window_list:
        window_msg.header.stamp = rospy.get_rostime()
        for i in range(int(window[0]), int(window[1])):
            pose = Pose()
            pose.position.x = 0
            pose.position.y = 0
            pose.position.z = 0
            pose.orientation.z = math.sin(
                math.radians(i * sector_angle) / 2)
            pose.orientation.w = math.cos(
                math.radians(i * sector_angle) / 2)
            window_msg.poses.append(pose)
    window_pub.publish(window_msg)

    # Initialize best valley array
    best_valley = []
    # The furthest a valley could every be from is 360 degrees since this is a circle
    best_distance = 361

    # Checking for each window we can drive through, which window is closest to the target angle
    # (distance is more like angle in this case)
    for i in range(len(window_list)):
        dist = get_target_distance(
            window_list[i][0], window_list[i][1], target, sector_angle)
        if dist < best_distance:
            best_valley = window_list[i]
            best_distance = dist
    return best_valley

# Gets the best angle to navigate to


def get_navigation_angle(
        target: int,
        threshold: float,
        data: LaserScan,
        smoothing_constant: float = 3) -> float:

    sector_angle = math.degrees(data.angle_increment)

    # Get the best valley in the LIDAR data given our target angle
    best_valley = get_valley(
        target,
        sector_angle,
        threshold,
        data,
        smoothing_constant)

    # If the rover is completely surrounded by obstacles, we want to turn hard right
    if len(best_valley) == 0:
        rospy.loginfo("Too many obstacles, must turn right")
        return 0

    # print("best valley: " + str(best_valley[0]) + " " + str(best_valley[1]))

    # Define the difference between 'wide' and 'narrow' valleys
    # For wide valleys, we want to drive on the edge of the valley
    #   i.e. Don't drive to the middle of a field just to avoid a pole
    # For narrow valleys, we want to drive in the middle of a valley
    #   i.e. We don't want to cut it too close on a doorway

    # research suggest that the biggest valley should not be bigger than 90
    # degrees
    max_valley = int(90 / sector_angle)
    # If the target is already in the best valley...
    if get_target_distance(
            best_valley[0],
            best_valley[1],
            target,
            sector_angle) == 0:

        # Report the current target angle; no adjustment needed
        #print("target * sector_angle = " + str(target * sector_angle))
        rospy.loginfo("In target valley")
        return target * sector_angle

    # If the valley is wide...
    elif is_wide_valley(best_valley[0], best_valley[1], max_valley):
        # Follow the protocol as defined above for wide valleys

        # Get the nearest edge of the sector
        nearest_sector = best_valley[1] if target > best_valley[1] else best_valley[0]

        # Construct a second border edge to make the valley of size max_valley
        border_sector = best_valley[1] - \
            max_valley if target > best_valley[1] else best_valley[0] + max_valley
        # Ensure that this new border edge is within the bounds of allowable sectors
        border_sector = 0 if border_sector < 0 else border_sector
        border_sector = len(data.ranges) if border_sector > len(
            data.ranges) else border_sector

        # Aim for the center of this new max_valley valley (this helps avoid accidentally clipping an edge of the robot)
        rospy.loginfo("Obstacle in the way, turning to wide valley")
        return ((nearest_sector + border_sector) / 2.0) * sector_angle

    # If the valley is narrow...
    else:
        # Follow the probotcol as defined above for narrow valleys

        # Aim for the center of the valley
        rospy.loginfo("Obstacle in the way, turning to narrow valley")
        return ((best_valley[0] + best_valley[1]) / 2.0) * sector_angle
