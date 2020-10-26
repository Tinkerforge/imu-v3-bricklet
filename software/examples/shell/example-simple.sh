#!/bin/sh
# Connects to localhost:4223 by default, use --host and --port to change this

uid=XYZ # Change XYZ to the UID of your IMU Bricklet 3.0

# Get current quaternion
tinkerforge call imu-v3-bricklet $uid get-quaternion
