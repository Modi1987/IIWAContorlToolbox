

# About

The folder "IIWAControlToolbox" contains the Sunrise.Workbench files for the server program on the robot side.
Copy the whole folder "IIWAControlToolbox" and paste it into your Sunrise.Workbench project (under src folder). Then, synchronize the project to the robot controller.
After synchronization, the application  "ICTServer" shall appear in the Applications list on the smartPad of the robot.

## Utilization:

To utilize the Toolbox, simply run the "ICTServer" application from the smartPad. Once the "ICTServer" application is running, you can connect to the robot from MATLAB (running on your external computer). Note that there is a time-out to connect to "ICTServer". When the application "ICTServer" is running, the user have one minute interval to connect from MATLAB, if a connection is not established during this interval, the application "ICTServer" will terminate automatically, and the user shall run it again before establishing a connction from MATLAB.


--------------------------------------

