Imports System
Imports Tinkerforge

Module ExampleSimple
    Const HOST As String = "localhost"
    Const PORT As Integer = 4223
    Const UID As String = "XYZ" ' Change XYZ to the UID of your IMU Bricklet 3.0

    Sub Main()
        Dim ipcon As New IPConnection() ' Create IP connection
        Dim imu As New BrickletIMUV3(UID, ipcon) ' Create device object

        ipcon.Connect(HOST, PORT) ' Connect to brickd
        ' Don't use device before ipcon is connected

        ' Get current quaternion
        Dim w, x, y, z As Short

        imu.GetQuaternion(w, x, y, z)

        Console.WriteLine("Quaternion [W]: " + (w/16383.0).ToString())
        Console.WriteLine("Quaternion [X]: " + (x/16383.0).ToString())
        Console.WriteLine("Quaternion [Y]: " + (y/16383.0).ToString())
        Console.WriteLine("Quaternion [Z]: " + (z/16383.0).ToString())

        Console.WriteLine("Press key to exit")
        Console.ReadLine()
        ipcon.Disconnect()
    End Sub
End Module
