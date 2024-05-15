#!/usr/bin/env python3

import rospy
import math
import actionlib
from wr_logic_longrange.msg import InitCompassAction, InitCompassGoal
from wrevolution.srv import SetHeading
from wr_drive_msgs.msg import DriveTrainCmd
from wr_hsi_sensing.msg import CoordinateMsg

## Timeout time for when we declare a long range navigation as failed
LONG_RANGE_TIMEOUT_TIME = rospy.Duration(2)


class InitCompassActionServer(object):
    def __init__(self, name) -> None:
        rospy.loginfo("initing compass set action server")
        # Publisher
        self.drive_pub = rospy.Publisher(
            rospy.get_param("~motor_speeds"), DriveTrainCmd, queue_size=10
        )

        # Subscribe to GPS Data
        self.subGPS = rospy.Subscriber(
            "/gps_coord_data", CoordinateMsg, self.update_gps_coord
        )

        self.current_lat = 0
        self.current_long = 0
        self.coord_time = 0

        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            InitCompassAction,
            execute_cb=self.execute_callback,
            auto_start=False,
        )
        self._as.start()

    # update current position based on gps coordinates
    def update_gps_coord(self, msg: CoordinateMsg) -> None:
        self.current_lat = msg.latitude
        self.current_long = msg.longitude
        self.coord_time = rospy.get_time()

    def execute_callback(self, goal: InitCompassGoal):
        rate = rospy.Rate(10)
        drive_msg = DriveTrainCmd(
            left_value=0.3,
            right_value=0.3,
        )

        # Wait for subscriber to get data
        self.rate = 100
        r = rospy.Rate(self.rate)
        while self.coord_time == 0:
            r.sleep()

        # Get current lat and long before moving
        lat_before = self.current_lat
        long_before = self.current_long
        rospy.loginfo("Before: " + str(self.current_lat) + " " + str(self.current_long))
        # Driving forward for 2 seconds
        start_time = rospy.get_rostime()
        while (
            rospy.get_rostime() - start_time < LONG_RANGE_TIMEOUT_TIME
            and not rospy.is_shutdown()
        ):
            rate.sleep()
            self.drive_pub.publish(drive_msg)
            rospy.loginfo(drive_msg)
        self.drive_pub.publish(0, 0)
        rospy.loginfo("MOTORS STOPPED")
        rospy.loginfo("Before: " + str(self.current_lat) + " " + str(self.current_long))

        # TODO set failed if current coordinate is too old (check coord_time)
        angle_heading = calculate_angle(
            long_before, lat_before, self.current_long, self.current_lat
        )
        success = set_heading_client(angle_heading)
        if success:
            rospy.loginfo("Heading set successfully!")
        else:
            rospy.loginfo("Failed to set heading.")

        return self._as.set_succeeded()


def set_heading_client(heading):
    rospy.wait_for_service("/pigeon/set_heading")
    try:
        set_heading = rospy.ServiceProxy("/pigeon/set_heading", SetHeading)
        return set_heading(heading)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed:", e)


def calculate_angle(x1, y1, x2, y2):
    """
    Calculate the angle (in degrees) between two sets of coordinates in math coordinates.
    """
    # Calculate the differences in x and y coordinates
    dx = x2 - x1
    dy = y2 - y1

    # Calculate the angle using arctan2, convert from radians to degrees
    angle_radians = math.atan2(dy, dx)
    angle_degrees = math.degrees(angle_radians)

    # Ensure angle is between 0 and 360 degrees
    if angle_degrees < 0:
        angle_degrees += 360

    return angle_degrees


if __name__ == "__main__":
    rospy.init_node("calculate_init_heading")
    initComp = InitCompassActionServer("InitCompassActionServer")
    rospy.spin()
