package frc.robot;


public class RobotMap {

    public class MotorPowerValues{
        //percent values for each motor to run in a comman, (0 - 1) 0r (0 - (-1))
        public static final double shooterWheelPercentOuput = 1;//0.85 for regular shot, 1 for full court shot

        public static final double shooterRPMValue = 5550; //works for both the line and the trench, tho must move hood//6650

        public static final double shooterRPMLineValue = 6650; 
    
        public static final double groundHarvestorIn = -1;
    
        public static final double groundHarvestorReverse = 1;
    
        public static final double spindexerSpinPower = -0.7;

        public static final double spindexerSpinConstantPower = -0.2;//0.2

        public static final double spindexerSpinAceleratedConstantPower = -0.3;

        public static final double spindexerSpinReversePower = 0.6;
    
        public static final double feedToShooter = 0.4;

        public static final double feedToShoooterReverse = -0.09;//-0.09
    }

    public class SpindexerValues {
        public static final int spindexerWaitForSpeed = 25; // number of command iterations to wait for spindexer to get up to speed
        public static final double spindexerJamVelocityCutoff = 10.0; // if velocity is below this value, assume spindexer is jammed
    }

    public class CANIDs{
        public static final int DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_ENCODER = 2;
        public static final int DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_MOTOR = 6;
        public static final int DRIVETRAIN_FRONT_LEFT_MODULE_DRIVE_MOTOR = 5;
    
        public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_ENCODER = 1;
        public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_MOTOR = 8;
        public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_DRIVE_MOTOR = 7;
    
        public static final int DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_ENCODER = 3;
        public static final int DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_MOTOR = 2;
        public static final int DRIVETRAIN_BACK_LEFT_MODULE_DRIVE_MOTOR = 1;
    
        public static final int DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_ENCODER = 0;
        public static final int DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_MOTOR = 4;
        public static final int DRIVETRAIN_BACK_RIGHT_MODULE_DRIVE_MOTOR = 3;
    
        public static final int shooter1 = 14;

        public static final int shooter2 = 13;

        public static final int HarvestorMotor = 15;

        public static final int feedingToShooterMotor = 16;
    
        public static final int spindexerSpinningMotor = 17;

        public static final int limitSwitchPort = 5;

        public static final int magSensorPortSpindexer = 5;

        public static final int magSensorPortHood = 6;

        public static final int revSpinBoreEnc_DIO_Blue = 2;//Spindexer central rod sensor
        public static final int revSpinBoreEnc_DIO_Yellow = 1;//Spindexer central rod sensor
    }

    public static final class ShooterConstants{
        public static final double encToRPM = 0.815;//((((1)*10*60)/2048)*(30/18))*(6.68/4);
    }


    public static final class LimelightConstants{

        //About: Set the camera height and goal height for distance calculations 
        public static final double cameraHeight = 2.583; //Camera height in meters
        public static final double goalHeight = 7.5; //Hatch panel height in meters
        public static final double mountAngle = 0.48869219056; //28 degrees
        public static final double LimeLightDrivePID = 0.002;  //pid value for turning in Limelight drive

    
    }
}
