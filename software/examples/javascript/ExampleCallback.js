var Tinkerforge = require('tinkerforge');

var HOST = 'localhost';
var PORT = 4223;
var UID = 'XYZ'; // Change XYZ to the UID of your IMU Bricklet 3.0

var ipcon = new Tinkerforge.IPConnection(); // Create IP connection
var imu = new Tinkerforge.BrickletIMUV3(UID, ipcon); // Create device object

ipcon.connect(HOST, PORT,
    function (error) {
        console.log('Error: ' + error);
    }
); // Connect to brickd
// Don't use device before ipcon is connected

ipcon.on(Tinkerforge.IPConnection.CALLBACK_CONNECTED,
    function (connectReason) {
        // Set period for quaternion callback to 0.1s (100ms)
        imu.setQuaternionCallbackConfiguration(100, false);
    }
);

// Register quaternion callback
imu.on(Tinkerforge.BrickletIMUV3.CALLBACK_QUATERNION,
    // Callback function for quaternion callback
    function (w, x, y, z) {
        console.log('Quaternion [W]: ' + w/16383.0);
        console.log('Quaternion [X]: ' + x/16383.0);
        console.log('Quaternion [Y]: ' + y/16383.0);
        console.log('Quaternion [Z]: ' + z/16383.0);
        console.log();
    }
);

console.log('Press key to exit');
process.stdin.on('data',
    function (data) {
        ipcon.disconnect();
        process.exit(0);
    }
);
