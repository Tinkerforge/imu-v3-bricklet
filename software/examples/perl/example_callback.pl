#!/usr/bin/perl

use strict;
use Tinkerforge::IPConnection;
use Tinkerforge::BrickletIMUV3;

use constant HOST => 'localhost';
use constant PORT => 4223;
use constant UID => 'XYZ'; # Change XYZ to the UID of your IMU Bricklet 3.0

# Callback subroutine for quaternion callback
sub cb_quaternion
{
    my ($w, $x, $y, $z) = @_;

    print "Quaternion [W]: " . $w/16383.0 . "\n";
    print "Quaternion [X]: " . $x/16383.0 . "\n";
    print "Quaternion [Y]: " . $y/16383.0 . "\n";
    print "Quaternion [Z]: " . $z/16383.0 . "\n";
    print "\n";
}

my $ipcon = Tinkerforge::IPConnection->new(); # Create IP connection
my $imu = Tinkerforge::BrickletIMUV3->new(&UID, $ipcon); # Create device object

$ipcon->connect(&HOST, &PORT); # Connect to brickd
# Don't use device before ipcon is connected

# Register quaternion callback to subroutine cb_quaternion
$imu->register_callback($imu->CALLBACK_QUATERNION, 'cb_quaternion');

# Set period for quaternion callback to 0.1s (100ms)
$imu->set_quaternion_callback_configuration(100, 0);

print "Press key to exit\n";
<STDIN>;
$ipcon->disconnect();
