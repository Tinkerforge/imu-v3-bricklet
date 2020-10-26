<?php

require_once('Tinkerforge/IPConnection.php');
require_once('Tinkerforge/BrickletIMUV3.php');

use Tinkerforge\IPConnection;
use Tinkerforge\BrickletIMUV3;

const HOST = 'localhost';
const PORT = 4223;
const UID = 'XYZ'; // Change XYZ to the UID of your IMU Bricklet 3.0

// Callback function for all data callback
function cb_allData($acceleration, $magnetic_field, $angular_velocity, $euler_angle,
                    $quaternion, $linear_acceleration, $gravity_vector, $temperature,
                    $calibration_status)
{
    echo "Acceleration [X]: " . $acceleration[0]/100.0 . " m/s²\n";
    echo "Acceleration [Y]: " . $acceleration[1]/100.0 . " m/s²\n";
    echo "Acceleration [Z]: " . $acceleration[2]/100.0 . " m/s²\n";
    echo "Magnetic Field [X]: " . $magnetic_field[0]/16.0 . " µT\n";
    echo "Magnetic Field [Y]: " . $magnetic_field[1]/16.0 . " µT\n";
    echo "Magnetic Field [Z]: " . $magnetic_field[2]/16.0 . " µT\n";
    echo "Angular Velocity [X]: " . $angular_velocity[0]/16.0 . " °/s\n";
    echo "Angular Velocity [Y]: " . $angular_velocity[1]/16.0 . " °/s\n";
    echo "Angular Velocity [Z]: " . $angular_velocity[2]/16.0 . " °/s\n";
    echo "Euler Angle [Heading]: " . $euler_angle[0]/16.0 . " °\n";
    echo "Euler Angle [Roll]: " . $euler_angle[1]/16.0 . " °\n";
    echo "Euler Angle [Pitch]: " . $euler_angle[2]/16.0 . " °\n";
    echo "Quaternion [W]: " . $quaternion[0]/16383.0 . "\n";
    echo "Quaternion [X]: " . $quaternion[1]/16383.0 . "\n";
    echo "Quaternion [Y]: " . $quaternion[2]/16383.0 . "\n";
    echo "Quaternion [Z]: " . $quaternion[3]/16383.0 . "\n";
    echo "Linear Acceleration [X]: " . $linear_acceleration[0]/100.0 . " m/s²\n";
    echo "Linear Acceleration [Y]: " . $linear_acceleration[1]/100.0 . " m/s²\n";
    echo "Linear Acceleration [Z]: " . $linear_acceleration[2]/100.0 . " m/s²\n";
    echo "Gravity Vector [X]: " . $gravity_vector[0]/100.0 . " m/s²\n";
    echo "Gravity Vector [Y]: " . $gravity_vector[1]/100.0 . " m/s²\n";
    echo "Gravity Vector [Z]: " . $gravity_vector[2]/100.0 . " m/s²\n";
    echo "Temperature: $temperature °C\n";
    echo "Calibration Status: " . sprintf("%08b", $calibration_status) . "\n";
    echo "\n";
}

$ipcon = new IPConnection(); // Create IP connection
$imu = new BrickletIMUV3(UID, $ipcon); // Create device object

$ipcon->connect(HOST, PORT); // Connect to brickd
// Don't use device before ipcon is connected

// Register all data callback to function cb_allData
$imu->registerCallback(BrickletIMUV3::CALLBACK_ALL_DATA, 'cb_allData');

// Set period for all data callback to 0.1s (100ms)
$imu->setAllDataCallbackConfiguration(100, FALSE);

echo "Press ctrl+c to exit\n";
$ipcon->dispatchCallbacks(-1); // Dispatch callbacks forever

?>
