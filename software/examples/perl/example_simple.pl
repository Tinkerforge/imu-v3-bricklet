#!/usr/bin/perl

use strict;
use Tinkerforge::IPConnection;
use Tinkerforge::BrickletIMUV3;

use constant HOST => 'localhost';
use constant PORT => 4223;
use constant UID => 'XYZ'; # Change XYZ to the UID of your IMU Bricklet 3.0

my $ipcon = Tinkerforge::IPConnection->new(); # Create IP connection
my $imu = Tinkerforge::BrickletIMUV3->new(&UID, $ipcon); # Create device object

$ipcon->connect(&HOST, &PORT); # Connect to brickd
# Don't use device before ipcon is connected

# Get current quaternion
my ($w, $x, $y, $z) = $imu->get_quaternion();

print "Quaternion [W]: " . $w/16383.0 . "\n";
print "Quaternion [X]: " . $x/16383.0 . "\n";
print "Quaternion [Y]: " . $y/16383.0 . "\n";
print "Quaternion [Z]: " . $z/16383.0 . "\n";

print "Press key to exit\n";
<STDIN>;
$ipcon->disconnect();
