package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.math.Vector2;

public class DriveCommand extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private XboxController controller;

    public DriveCommand(DrivetrainSubsystem drivetrain, XboxController controller) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.controller = controller;
    }

    @Override
    public void execute() {
        //sets these valuse to be eqaul to their respective sticks
        double strafe = -controller.getRawAxis(1);
        double forward = -controller.getRawAxis(0);
        double rotation;

        if(Robot.getRobotContainer().getDriverController().getRawButton(3))
        {
            //if button is pressed then use limlight drive as rotation value
            rotation = RobotContainer.limelight.rotatetoTarget(RobotMap.LimelightConstants.LimeLightDrivePID);

            //turns the limelight on
            Limelight.enableTracking();
        } else {

            //else the stick is the value
            //but is has to be caluculated to give the right numbers
            double rotationRaw = -controller.getRawAxis(4)*0.3;
            rotation = -0.5 * Math.pow(rotationRaw, 2) * Math.signum(rotationRaw);    
            
            //turns the limelight off to comply to rule
            Limelight.disableTracking();
        }
        

        //finnaly this creates deadzones for the sticks
        //so if the value is less than this number it reads as 0
        if (Math.abs(forward) < .095) {
            forward = 0;
        }

        if (Math.abs(strafe) < .095) {
            strafe = 0;
        }

        //if (Math.abs(rotation) < .005) {
        //    rotation = 0;
        //}


        //creates the final drive command
        drivetrain.drive(new Vector2(strafe, forward), rotation, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}