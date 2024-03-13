#!/usr/bin/env python

## @file
# @brief Node executing GPS and heading calculations
# @ingroup wr_hsi_sensing
# @defgroup wr_hsi_sensing_gps GPS
# @brief Responsible for calculating GPS and heading data
# @details Getting coordinate data from the GPS is a straightforward query to the sensor.  Calculating the heading from GPS data requires
# reading multiple GPS positions and finding the heading of the line connecting the two GPS coordinates.  This method works well for systems that
# have consistent motion patterns and can really only move in 1 direction (like cars/trucks).  This is not a great candidate for calculating heading on
# the rover; it is a better idea to get the @ref wr_hsi_sensing_imu working.
# @ingroup wr_hsi_sensing
# @{

import math
import rospy
from wr_hsi_sensing.msg import CoordinateMsg
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix


# Constants to vary
## Rate at which pose messages are published in hz
f = 100

## @cond
rospy.init_node("gps_testing", anonymous=False)
## @endcond
## Rate controlling how frequently we publish data
rate = rospy.Rate(f)
## Publisher for GPS coordinate data
gpsPub = rospy.Publisher("/gps_coord_data", CoordinateMsg, queue_size=1)
## Publisher for heading data
# TODO(@bennowotny): This should be migrated back to the IMU code
headingPub = rospy.Publisher("/heading_data", Float64, queue_size=1)

# ## Last latitude recorded, used to calculate heading
# last_lat = None
# ## Last longitude recorded, used to calculate heading
# last_long = None
# ## Last heading recorded, used to maintain publishing frequency once heading is locked
# cached_heading = None

## How much the GPS data has to deviate to reliably calculate heading
GPS_TOLERANCE = 0.000001


class MovingAverage:
    """
    Class for keeping track of moving average of data. Implemented
    using a circular buffer
    """
    def __init__(self, windowSize: int):
        self.windowSize = max(windowSize, 1)  # don't allow to be less than 1
        self.data = []
        self.avg = -1
        self.index = 0

    def add(self, data: float) -> None:
        """
        Adds data to the circular buffer. Simply appends if buffer not at max size, otherwise
        it starts overwriting data
        """
        if len(self.data) < self.windowSize:  # buffer not at max size, so just increase size
            self.data.append(data)
            self.index += 1
        else:
            self.data[self.index] = data
            self.index = (self.index + 1) % self.windowSize  # wrap around for next write
        self.avg = sum(self.data) / len(self.data)

    def get(self) -> float:
        """
        Returns average of items in circular buffer
        """
        return self.avg
    
    def getLatest(self) -> float:
        """
        returns the last recorded data, not an average
        """
        return self.data[self.index]


class GPSController:
    """
    Controls gps signals. Handles data updates and publishing data
    """
    def __init__(self, gpsPub: rospy.Publisher, headingPub: rospy.Publisher, movingAverageWindow: int, debug: bool = False):
        self.headingPub = headingPub
        self.gpsPub = gpsPub
        
        self.averageHeading = True

        self.latQueue = MovingAverage(movingAverageWindow)
        self.longQueue = MovingAverage(movingAverageWindow)
        self.prevLat = 0
        self.prevLong = 0

        self.heading = 0
        self.publishHeading = True

        self.debug = debug

    def udpateHeading(self) -> None:
        """
        Updates heading according to formula explained here:
        https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
        TODO: this function should not be used, switch back to IMU for heading
        """
        if (self.latQueue.get() is not None and self.longQueue.get() is not None and (
            abs(self.latQueue.get() - self.prevLat) > GPS_TOLERANCE
            or abs(self.longQueue.get() - self.lastLong))
        ):
            # Find heading from the change in latitude and longitude
            X = math.cos(self.latQueue.get()) * math.sin(self.longQueue.get() - self.prevLong)
            Y = math.cos(self.prevLat) * math.sin(self.latQueue.get()) - math.sin(
                self.prevLat
            ) * math.cos(self.latQueue.get()) * math.cos(self.longQueue.get() - self.prevLong)
            bearing = math.atan2(X, Y)  # probably correct, not verified though
            bearing = bearing if bearing > 0 else 2 * math.pi + bearing
            bearing = math.degrees(bearing)

            self.heading = bearing


    def readCallback(self, msg):
        self.prevLat = self.latQueue.get()
        self.prevLong = self.longQueue.get()

        self.latQueue.add(msg.latitude)
        self.longQueue.add(msg.longitude)

        if self.debug:
            print("latitude  from sensor: {}".format(msg.latitude))
            print("longitude from sensor: {}".format(msg.longitude))


    def writeCallback(self) -> None:
        """
        Callback function called by ros. Updates headings and publishes data
        """
        
        if self.latQueue.get() is not None and self.longQueue.get() is not None:
            self.updateHeading()

            # publish lat and long
            msg = CoordinateMsg(latitude=self.latQueue.get(), longitude=self.longQueue.get())
            self.gpsPub.publish(msg)

            # publish heading if desired
            if self.publishHeading and self.heading is not None:
                self.headingPub.publish(Float64(data=self.heading))

            # debug message
            if self.debug:
                print(f"DATA PUBLISHED    lat: {self.latQueue.get()}  long: {self.longQueue.get()}  heading: {self.heading}")



if __name__ == "__main__":
    ## GPS used to get data, defined on the RPi I2C bus
    controller = GPSController(gpsPub, headingPub, 1)

    # set up subscriber to ros library for the gps sensor
    rospy.init_node("gps_data_subscriber_node", anonymous = False)
    rospy.Subscriber('/gps/fix', NavSatFix, controller.readCallback)

    ## ROS timer to calculate and publish GPS data
    timer = rospy.Timer(rate.sleep_dur, lambda _: controller.writeCallback()) 
    
    rospy.spin()

## @}
