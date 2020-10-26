<?php

require_once('Tinkerforge/IPConnection.php');
require_once('Tinkerforge/BrickletIMUV3.php');

use Tinkerforge\IPConnection;
use Tinkerforge\BrickletIMUV3;

const HOST = 'localhost';
const PORT = 4223;
const UID = 'XYZ'; // Change XYZ to the UID of your IMU Bricklet 3.0

$ipcon = new IPConnection(); // Create IP connection
$imu = new BrickletIMUV3(UID, $ipcon); // Create device object

$ipcon->connect(HOST, PORT); // Connect to brickd
// Don't use device before ipcon is connected

// Get current quaternion
$quaternion = $imu->getQuaternion();

echo "Quaternion [W]: " . $quaternion['w']/16383.0 . "\n";
echo "Quaternion [X]: " . $quaternion['x']/16383.0 . "\n";
echo "Quaternion [Y]: " . $quaternion['y']/16383.0 . "\n";
echo "Quaternion [Z]: " . $quaternion['z']/16383.0 . "\n";

echo "Press key to exit\n";
fgetc(fopen('php://stdin', 'r'));
$ipcon->disconnect();

?>
