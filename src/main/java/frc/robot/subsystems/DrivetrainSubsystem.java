package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.kinematics.ChassisVelocity;
import org.frcteam2910.common.kinematics.SwerveKinematics;
import org.frcteam2910.common.kinematics.SwerveOdometry;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DrivetrainSubsystem extends SubsystemBase {
    //creates the distance beween the swerve modules so it has more acurate turning
    private static final double TRACKWIDTH = 20;
    private static final double WHEELBASE = 25;


    //the offset of each module, look at the readme for how to set up
    //left=front, right = back
    //TO MAKE THE SWERVE BO ALLIGNED MAKE THESE ZERO
    //THEN ALIGN AND GET THE ALIGNED VALUES FROM SHUFFLEBOARD, PLACE THOSE IN CODE
    public static final double DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_OFFSET = -Math.toRadians(199.5);//199.9
    public static final double DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_OFFSET = -Math.toRadians(242.75);//167
    public static final double DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_OFFSET = -Math.toRadians(30.5);//31.6
    public static final double DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_OFFSET = -Math.toRadians(196.2);//195.7



    //tracks the current positon of modules for a table for auto navigation
    private static final DrivetrainSubsystem instance;
    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private final NetworkTable currentPoseTable = nt.getTable("/pathFollowing/current");
    private final NetworkTableEntry currentXEntry = currentPoseTable.getEntry("x");
    private final NetworkTableEntry currentYEntry = currentPoseTable.getEntry("y");
    private final NetworkTableEntry currentAngleEntry = currentPoseTable.getEntry("angle");

    //the scale of the robot moving and a timer for auto and math problems
    private final double X_SCALE = 1;
    private final double Y_SCALE = 1;
    private double lastUpdated = Timer.getFPGATimestamp();


    //this creates the four swerve modules with the two motors and the offset and its position on the robot
    //change to make it drive forward
    private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
                    .angleEncoder(new AnalogInput(RobotMap.CANIDs.DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_ENCODER),
                            DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_OFFSET)
                    .angleMotor(new CANSparkMax(RobotMap.CANIDs.DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless), Mk2SwerveModuleBuilder.MotorType.NEO)
                    .driveMotor(new CANSparkMax(RobotMap.CANIDs.DRIVETRAIN_FRONT_LEFT_MODULE_DRIVE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless), Mk2SwerveModuleBuilder.MotorType.NEO)
                    .build();
    private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
                    .angleEncoder(new AnalogInput(RobotMap.CANIDs.DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_ENCODER),
                            DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_OFFSET)
                    .angleMotor(new CANSparkMax(RobotMap.CANIDs.DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless), Mk2SwerveModuleBuilder.MotorType.NEO)
                    .driveMotor(new CANSparkMax(RobotMap.CANIDs.DRIVETRAIN_FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless), Mk2SwerveModuleBuilder.MotorType.NEO)
                    .build();
    private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
                    .angleEncoder(new AnalogInput(RobotMap.CANIDs.DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_ENCODER),
                            DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_OFFSET)
                    .angleMotor(new CANSparkMax(RobotMap.CANIDs.DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless), Mk2SwerveModuleBuilder.MotorType.NEO)
                    .driveMotor(new CANSparkMax(RobotMap.CANIDs.DRIVETRAIN_BACK_LEFT_MODULE_DRIVE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless), Mk2SwerveModuleBuilder.MotorType.NEO)
                    .build();
    private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
                    .angleEncoder(new AnalogInput(RobotMap.CANIDs.DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_ENCODER),
                            DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_OFFSET)
                    .angleMotor(new CANSparkMax(RobotMap.CANIDs.DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless), Mk2SwerveModuleBuilder.MotorType.NEO)
                    .driveMotor(new CANSparkMax(RobotMap.CANIDs.DRIVETRAIN_BACK_RIGHT_MODULE_DRIVE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless), Mk2SwerveModuleBuilder.MotorType.NEO)
                    .build();
    private final SwerveModule[] modules = { frontLeftModule, frontRightModule, backLeftModule, backRightModule };

    //the kinematics of were each module is in relation to each other
    private final SwerveKinematics kinematics = new SwerveKinematics(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // Front Left
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // Front Right
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // Back Left
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // Back Right
    );

    
    private final static HolonomicDriveSignal ZeroDriveSignal = new HolonomicDriveSignal(Vector2.ZERO, 0, false);
    private final SwerveOdometry odometry = new SwerveOdometry(kinematics,
            new RigidTransform2(new Vector2(TRACKWIDTH / 2, WHEELBASE / 2), Rotation2.fromDegrees(90)));

    private final Object sensorLock = new Object();
    private final NavX navX$sensorLock = new NavX(SPI.Port.kMXP);

    private final Object kinematicsLock = new Object();
    private RigidTransform2 pose$kinematicsLock = RigidTransform2.ZERO;

    private final Object stateLock = new Object();
    private HolonomicDriveSignal driveSignal$stateLock = null;

    private NetworkTableEntry[] moduleAngleEntries = new NetworkTableEntry[modules.length];

    static {
        instance = new DrivetrainSubsystem();
    }

    private DrivetrainSubsystem() {
        synchronized (sensorLock) {
            navX$sensorLock.setInverted(true);
        }

        ShuffleboardTab tab = Shuffleboard.getTab("testinfo");

        ShuffleboardLayout frontLeftModuleContainer = tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withPosition(1, 0).withSize(2, 3);
        moduleAngleEntries[0] = frontLeftModuleContainer.add("Angle", 0.0).getEntry();

        ShuffleboardLayout frontRightModuleContainer = tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withPosition(3, 0).withSize(2, 3);
        moduleAngleEntries[1] = frontRightModuleContainer.add("Angle", 0.0).getEntry();

        ShuffleboardLayout backLeftModuleContainer = tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withPosition(5, 0).withSize(2, 3);
        moduleAngleEntries[2] = backLeftModuleContainer.add("Angle", 0.0).getEntry();

        ShuffleboardLayout backRightModuleContainer = tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withPosition(7, 0).withSize(2, 3);
        moduleAngleEntries[3] = backRightModuleContainer.add("Angle", 0.0).getEntry();



    }

    public static DrivetrainSubsystem getInstance() {
        return instance;
    }

    /**
     * @return The current rate of change in yaw angle (in degrees per second)
     */
    public double getAngularVelocity() {
        synchronized (sensorLock) {
            return navX$sensorLock.getRate();
        }
    }

    public RigidTransform2 getPose() {
        synchronized (kinematicsLock) {
            return pose$kinematicsLock;
        }
    }

    public RigidTransform2 getScaledPose() {
        final var pose = getPose();
        final var translation = pose.translation.multiply(X_SCALE, Y_SCALE);
        final var rotation = pose.rotation.rotateBy(Rotation2.fromRadians(Math.PI / 2));

        return new RigidTransform2(translation, rotation);
    }

    public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean fieldOriented) {
        synchronized (stateLock) {
            driveSignal$stateLock = new HolonomicDriveSignal(translationalVelocity, -rotationalVelocity, fieldOriented);
        }
    }

    public void resetPose(Vector2 translation, Rotation2 angle) {
        System.out.println("Reset Pose");
        synchronized (kinematicsLock) {
            odometry.resetPose(
                    new RigidTransform2(new Vector2(translation.x / X_SCALE, translation.y / Y_SCALE), angle));
            resetGyroAngle(angle);
            pose$kinematicsLock = odometry.getPose();
        }
        updatePoseNT();
    }

    public void resetGyroAngle(Rotation2 angle) {
        System.out.println("Reset Gyro Angle");
        synchronized (sensorLock) {
            navX$sensorLock.setAdjustmentAngle(navX$sensorLock.getUnadjustedAngle().rotateBy(angle.inverse()));
        }
    }

    
    public void resetGyroscope() {
        navX$sensorLock.setAdjustmentAngle(navX$sensorLock.getUnadjustedAngle());
    }


    private void update() {
        final var now = Timer.getFPGATimestamp();
        final var dt = now - lastUpdated;
        lastUpdated = now;

        updateOdometry(dt);

        HolonomicDriveSignal driveSignal;
        synchronized (stateLock) {
            driveSignal = driveSignal$stateLock;
            driveSignal$stateLock = ZeroDriveSignal;
        }

        updateModules(driveSignal, dt);
    }

    private void updateOdometry(double dt) {
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.updateSensors();

            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getCurrentAngle()))
                    .scale(module.getCurrentVelocity());
        }

        Rotation2 angle;
        synchronized (sensorLock) {
            angle = navX$sensorLock.getAngle();
        }

        RigidTransform2 pose = odometry.update(angle, dt, moduleVelocities);

        synchronized (kinematicsLock) {
            pose$kinematicsLock = pose;
        }
    }

    private void updateModules(HolonomicDriveSignal signal, double dt) {
        ChassisVelocity velocity;
        if (signal == null) {
            velocity = new ChassisVelocity(Vector2.ZERO, 0.0);
        } else if (signal.isFieldOriented()) {
            velocity = new ChassisVelocity(signal.getTranslation().rotateBy(getPose().rotation.inverse()),
                    signal.getRotation());
        } else {
            velocity = new ChassisVelocity(signal.getTranslation(), signal.getRotation());
        }

        Vector2[] moduleOutputs = kinematics.toModuleVelocities(velocity);
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1.0);

        for (int i = 0; i < modules.length; i++) {
            final var module = modules[i];
            module.setTargetVelocity(moduleOutputs[i]);
            module.updateState(dt);
        }
    }

    private void updatePoseNT() {
        final var pose = getScaledPose();

        currentAngleEntry.setDouble(pose.rotation.toRadians());
        currentXEntry.setDouble(pose.translation.x);
        currentYEntry.setDouble(pose.translation.y);

    }

    @Override
    public void periodic() {
        updatePoseNT();
        update();
        
        SmartDashboard.putNumber("Front Left Module Angle", Math.toDegrees(frontLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Front Right Module Angle", Math.toDegrees(frontRightModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Left Module Angle", Math.toDegrees(backLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Right Module Angle", Math.toDegrees(backRightModule.getCurrentAngle()));
        SmartDashboard.putNumber("Hey Driver make sure to check controllers/ports, then correct agreed apon auto, then witch to CAMERA for view ", 1);
    }
}