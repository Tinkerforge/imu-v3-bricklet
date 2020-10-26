#!/bin/sh
# Connects to localhost:4223 by default, use --host and --port to change this

uid=XYZ # Change XYZ to the UID of your IMU Bricklet 3.0

# Handle incoming quaternion callbacks
tinkerforge dispatch imu-v3-bricklet $uid quaternion &

# Set period for quaternion callback to 0.1s (100ms)
tinkerforge call imu-v3-bricklet $uid set-quaternion-callback-configuration 100 false

echo "Press key to exit"; read dummy

kill -- -$$ # Stop callback dispatch in background
