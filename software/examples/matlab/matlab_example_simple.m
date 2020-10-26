function matlab_example_simple()
    import com.tinkerforge.IPConnection;
    import com.tinkerforge.BrickletIMUV3;

    HOST = 'localhost';
    PORT = 4223;
    UID = 'XYZ'; % Change XYZ to the UID of your IMU Bricklet 3.0

    ipcon = IPConnection(); % Create IP connection
    imu = handle(BrickletIMUV3(UID, ipcon), 'CallbackProperties'); % Create device object

    ipcon.connect(HOST, PORT); % Connect to brickd
    % Don't use device before ipcon is connected

    % Get current quaternion
    quaternion = imu.getQuaternion();

    fprintf('Quaternion [W]: %g\n', quaternion.w/16383.0);
    fprintf('Quaternion [X]: %g\n', quaternion.x/16383.0);
    fprintf('Quaternion [Y]: %g\n', quaternion.y/16383.0);
    fprintf('Quaternion [Z]: %g\n', quaternion.z/16383.0);

    input('Press key to exit\n', 's');
    ipcon.disconnect();
end
