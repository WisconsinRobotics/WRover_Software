#!/usr/bin/env python

## @file
# @brief Node executing GPS and heading calculations
# @ingroup wr_hsi_sensing
# @defgroup wr_hsi_sensing_imu IMU
# @brief Responsible for calculating GPS and heading data
# @details This code is currently not executing, it was disabled due to sensor failure at URC 2023
# Using the IMU's magnetometer, we calculate the heading of the robot by using trigonometry from the magnetic sensor readings, similar to a compass.
# It is, ultimately, the significantly better method of getting heading data.
#
# This program calculates the IMU heading by reading raw magnetometer data, normalizing and offsetting it (accounting for hard- and soft-iron calibration),
# and then using trigonometric functions to transform x and y field strengths into an angle (atan).  Due to sensor noise on the raw
# magnetometer readings, we use a band-pass method to only accept readings that are 'close' to the previous reading.  To normalize the data,
# the program keeps track of the min/max strengths of the x/y axes and scales the current reading to be on the interval [-1, 1] to adjust for non-uniform
# strength offsets.  Keep in mind that this method assumes a flat orientation for the IMU.
# @ingroup wr_hsi_sensing
# @{

from typing import Optional
import rospy
from wr_hsi_sensing.BNO055 import BNO055
from std_msgs.msg import Float64
import math

## Constants to vary
## Rate at which pose messages are published in hz
f = 100

## @cond
rospy.init_node("imu_testing", anonymous=False)
## @endcond
## ROS rate to control publishing frequency
rate = rospy.Rate(f)
## Publisher for magnetic strength in the x-axis
pub_x = rospy.Publisher("/mag_x", Float64, queue_size=1)
## Publisher for magnetic field strength in the y-axis
pub_y = rospy.Publisher("/mag_y", Float64, queue_size=1)
## Publisher for magnetic field for strength in the z-axis
pub_z = rospy.Publisher("/mag_z", Float64, queue_size=1)

## Publisher for normalized magnetic field strength in the x-axis
pub_norm_x = rospy.Publisher("/norm_mag_x", Float64, queue_size=1)
## Publisher for normalized magnetic field strenth in the y-axis
pub_norm_y = rospy.Publisher("/norm_mag_y", Float64, queue_size=1)
## Publisher for calculated magnetic heading, in degrees
pub_heading = rospy.Publisher("/heading_data", Float64, queue_size=1)

## The upper bound on measurement deviation to be considered a valid reading.
#
# If the sensor measurement deviates more than this many units between readings, the new reading is rejected as noise.
MAG_NOISE_THRESH = 1000


## Calculates a moving average based on fed values
#
# This class provides an equally weighted moving average for a fixed number of values 'fed' to the instance
class MovingAverage:
    ## How many past values (at most) to be included in any particular moving average
    _AVERAGING_WINDOW_SIZE = 15

    def __init__(self):
        """Constructs a @ref imu_testing.MovingAverage filter

        Assumes that no values have been fed to the filter
        """
        ## The list of values to include in the filter
        self._values = []

    def feed(self, value) -> None:
        """Pushes a new value into the filter

        If the filter is already at capacity, then the oldest value is removed from the filter

        @param value The new value to insert into the filter
        """
        self._values.insert(0, value)
        if len(self._values) > MovingAverage._AVERAGING_WINDOW_SIZE:
            self._values.pop()

    def get_value(self) -> Optional[float]:
        """Get the current moving average

        @return The arithmetic average of the values tracked in the filter.  If no values are tracked, then None
        """
        return sum(self._values) / len(self._values) if len(self._values) > 0 else None


## The lowest recorded x-axis magnetic field strength
#
# Initial value chosen to naturally acquired correct value after first comparison
min_x = float("inf")
## The highest recorded x-axis magnetic field strength
#
# Initial value chosen to naturally acquired correct value after first comparison
max_x = float("-inf")
## The lowest recorded y-axis magnetic field strength
#
# Initial value chosen to naturally acquired correct value after first comparison
min_y = float("inf")
## The highest recorded y-axis magnetic field strength
#
# Initial value chosen to naturally acquired correct value after first comparison
max_y = float("-inf")
## Previously read magnetic field strength on the x-axis
prev_x = 0
## Previously read magnetic field strength on the y-axis
prev_y = 0
## The moving average filter for x-axis readings
x_filter = MovingAverage()
## The moving average filter for the y-axis readings
y_filter = MovingAverage()
## Has the x-axis reached initialization yet?
x_has_init = False
## Has the y-axis reached initialization yet?
y_has_init = False

# Create IMU Sensor object
## The driver to communicate with the IMU over I2C
sensor = BNO055()
if not sensor.begin(mode=BNO055.OPERATION_MODE_MAGONLY):
    raise RuntimeError("IMU Failed to initialize")
sensor.setExternalCrystalUse(True)
# Dump some initial IMU readings
# imuInitCounter = 0
# while((sensor.getVector(BNO055.VECTOR_EULER)[2] == 0.0)and
#         (imuInitCounter > 100)):
#     print(sensor.getVector(BNO055.VECTOR_EULER))
#     rate.sleep()
#     imuInitCounter+=1


def cb():
    """ROS callback to calculate and publish IMU heading data"""
    global max_x
    global min_x
    global max_y
    global min_y
    global prev_x
    global prev_y
    global x_filter
    global y_filter
    global x_has_init
    global y_has_init

    try:
        mag_z = int.from_bytes(
            sensor.readBytes(BNO055.BNO055_MAG_DATA_Z_LSB_ADDR, 2), "little"
        )
        mag_x = int.from_bytes(
            sensor.readBytes(BNO055.BNO055_MAG_DATA_X_LSB_ADDR, 2), "little"
        )
        mag_y = int.from_bytes(
            sensor.readBytes(BNO055.BNO055_MAG_DATA_Y_LSB_ADDR, 2), "little"
        )
    except:
        return

    mag_x = mag_x if mag_x < 32768 else mag_x - 65536
    mag_y = mag_y if mag_y < 32768 else mag_y - 65536
    mag_z = mag_z if mag_z < 32768 else mag_z - 65536

    if not x_has_init and mag_x == 0:
        return
    x_has_init = True
    if not y_has_init and mag_y == 0:
        return
    y_has_init = True

    # TODO: Consider differential noise filtering (prev_msg) or absolute
    if abs(mag_x - prev_x) < MAG_NOISE_THRESH:
        x_filter.feed(mag_x)

    if abs(mag_y - prev_y) < MAG_NOISE_THRESH:
        y_filter.feed(mag_y)

    # rospy.loginfo(f"min/max x: {min_x}, {max_x}")
    # rospy.loginfo(f"min/max y: {min_y}, {max_y}")

    mag_x = x_filter.get_value()
    mag_y = y_filter.get_value()

    if mag_x is None or mag_y is None:
        return

    max_x = max(max_x, mag_x)
    min_x = min(min_x, mag_x)
    max_y = max(max_y, mag_y)
    min_y = min(min_y, mag_y)
    prev_x = mag_x
    prev_y = mag_y

    range_x = max_x - min_x
    range_y = max_y - min_y
    norm_x = mag_x - (range_x / 2 + min_x)
    norm_y = mag_y - (range_y / 2 + min_y)
    norm_x /= range_x if range_x != 0 else 1
    norm_y /= range_y if range_y != 0 else 1
    pub_norm_x.publish(Float64(norm_x))
    pub_norm_y.publish(Float64(norm_y))

    heading = math.atan2(norm_y, norm_x) + math.pi
    heading = math.degrees(heading)
    pub_heading.publish(Float64(heading))

    print(heading)

    mag_z = mag_z if abs(mag_z) < 10000 else 0

    pub_x.publish(mag_x)
    pub_y.publish(mag_y)
    pub_z.publish(float(mag_z))


if __name__ == "__main__":
    prev_x = int.from_bytes(
        sensor.readBytes(BNO055.BNO055_MAG_DATA_X_LSB_ADDR, 2), "little"
    )
    prev_y = int.from_bytes(
        sensor.readBytes(BNO055.BNO055_MAG_DATA_Y_LSB_ADDR, 2), "little"
    )
    prev_x = prev_x if prev_x < 32768 else prev_x - 65536
    prev_y = prev_y if prev_y < 32768 else prev_y - 65536
    timer = rospy.Timer(rate.sleep_dur, lambda _: cb())
    rospy.spin()
## @}
