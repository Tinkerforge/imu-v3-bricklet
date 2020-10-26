#!/bin/sh
# Connects to localhost:4223 by default, use --host and --port to change this

uid=XYZ # Change XYZ to the UID of your IMU Bricklet 3.0

# Handle incoming all data callbacks
tinkerforge dispatch imu-v3-bricklet $uid all-data &

# Set period for all data callback to 0.1s (100ms)
tinkerforge call imu-v3-bricklet $uid set-all-data-callback-configuration 100 false

echo "Press key to exit"; read dummy

kill -- -$$ # Stop callback dispatch in background
