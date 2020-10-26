#!/usr/bin/env ruby
# -*- ruby encoding: utf-8 -*-

require 'tinkerforge/ip_connection'
require 'tinkerforge/bricklet_imu_v3'

include Tinkerforge

HOST = 'localhost'
PORT = 4223
UID = 'XYZ' # Change XYZ to the UID of your IMU Bricklet 3.0

ipcon = IPConnection.new # Create IP connection
imu = BrickletIMUV3.new UID, ipcon # Create device object

ipcon.connect HOST, PORT # Connect to brickd
# Don't use device before ipcon is connected

# Get current quaternion as [w, x, y, z]
quaternion = imu.get_quaternion

puts "Quaternion [W]: #{quaternion[0]/16383.0}"
puts "Quaternion [X]: #{quaternion[1]/16383.0}"
puts "Quaternion [Y]: #{quaternion[2]/16383.0}"
puts "Quaternion [Z]: #{quaternion[3]/16383.0}"

puts 'Press key to exit'
$stdin.gets
ipcon.disconnect
