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

# Register quaternion callback
imu.register_callback(BrickletIMUV3::CALLBACK_QUATERNION) do |w, x, y, z|
  puts "Quaternion [W]: #{w/16383.0}"
  puts "Quaternion [X]: #{x/16383.0}"
  puts "Quaternion [Y]: #{y/16383.0}"
  puts "Quaternion [Z]: #{z/16383.0}"
  puts ''
end

# Set period for quaternion callback to 0.1s (100ms)
imu.set_quaternion_callback_configuration 100, false

puts 'Press key to exit'
$stdin.gets
ipcon.disconnect
