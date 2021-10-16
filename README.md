# 4206 Talos

Features
-Swerve MK2 Modules code 
-Limelight Targeting
-Ajustable Servo code for hood
-Autonomous Code for Swerve 

## Configure For Your Robot

0. Build the Robot connected to school wifi to allow all online features

1. In the `DrivetrainSubsystem` class:
    2. Set all of the `*_ANGLE_OFFSET` constants to `-Math.toRadians(0.0)`.

2. Deploy the code to your robot.
    > NOTE: The robot isn't drivable quite yet, we still have to setup the module offsets

3. Turn the robot on its robotcart and align all the wheels so they are facing in the forwards direction.
    > NOTE: The wheels will be pointed forwards (not backwards) when modules are turned so the large bevel gears are towards the right side of the robot. When aligning the wheels they must be as straight as possible. It is recommended to use a long strait edge such as a piece of 2x1 in order to make the wheels straight.

4. Open Shuffleboard and open the file in the top left `swervebutcool` this holds the info for the wheels

5. Record the angles of each module using the angle put onto Shuffleboard. The values are named
    `Front Left Module Angle`, `Front Right Module Angle`, etc.

6. Set the values of the `*_ANGLE_OFFSET` to `-Math.toRadians(<the angle you recorded>)`
    > NOTE: All angles must be in degrees.

7. Re-deploy and try to drive the robot forwards. All the wheels should stay parallel to each other. If not go back to
    step 3.

8. Make sure all the wheels are spinning in the correct direction. If not, add 180 degrees to the offset of each wheel
    that is spinning in the incorrect direction. i.e `-Math.toRadians(<angle> + 180.0)`.
