#!/usr/bin/env python

import rospy
from BNO055 import BNO055
from adafruit_gps import I2C,GPS_GtopI2C as GPS
from std_msgs.msg import Float32
import time

## Constants to vary
f = 100     # Hz | Rate at which pose messages are published

rospy.init_node('ai_hardware_testing', anonymous=False)
rate = rospy.Rate(f)
test_pub_heading = rospy.Publisher('/test_heading_data', Float32, queue_size=1)

# Create IMU Sensor object
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

def init()-> None:
    while not rospy.is_shutdown():
        publish_heading()

## Retrieve heading from IMU (Returns a float angle)
def get_heading():
    # TODO: get actual obset from an absolute value ---- Build calibration method for startup.
    offset_from_east = 90 
    # imu returns clockwise values, we need it counterclockwise
    relative_angle = 360 - sensor.getVector(BNO055.VECTOR_EULER)[0]
    return (offset_from_east + relative_angle) % 360

def publish_heading():
    test_pub_heading.publish(get_heading())

if __name__ == '__main__':
    while True:
        print(f"mag_x = {int.from_bytes(sensor.readBytes(BNO055.BNO055_MAG_DATA_X_LSB_ADDR, 2), 'little')}")
        print(f"mag_y = {int.from_bytes(sensor.readBytes(BNO055.BNO055_MAG_DATA_Y_LSB_ADDR, 2), 'little')}")
        print(f"mag_z = {int.from_bytes(sensor.readBytes(BNO055.BNO055_MAG_DATA_Z_LSB_ADDR, 2), 'little')}")
        time.sleep(1)