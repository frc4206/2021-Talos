/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.autonomous.autocustomcommands.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EjectorSubsystem;
import frc.robot.subsystems.HarvestorSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;


//kevins favorite drive code
public class RobotContainer {
  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static HarvestorSubsystem harvestorSubsystem = new HarvestorSubsystem();
  public static SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem();
  public static EjectorSubsystem ejectorSubsystem = new EjectorSubsystem();
  public static HoodSubsystem hoodSubsystem = new HoodSubsystem();
  public static Limelight limelight = new Limelight();


  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);



  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    DrivetrainSubsystem.getInstance().setDefaultCommand(new DriveCommand(drivetrain, driverController));
    autoChooser.addOption("Drive Forward Only", new DriveForward(drivetrain));
    autoChooser.setDefaultOption("Trench", new TrenchAuto(drivetrain, shooterSubsystem, spindexerSubsystem, harvestorSubsystem, ejectorSubsystem, hoodSubsystem));
    autoChooser.setDefaultOption("Trench Offset", new TrenchOffsetAuto(drivetrain, shooterSubsystem, spindexerSubsystem, harvestorSubsystem, ejectorSubsystem));
    autoChooser.addOption("Three Ball Auto", new ThreeballAuto(drivetrain, shooterSubsystem, spindexerSubsystem, ejectorSubsystem));
    autoChooser.addOption("test Path Auto", new TestPathAuto(drivetrain));
    SmartDashboard.putData("Auto Selector", autoChooser);




    new JoystickButton(driverController, 1).whileHeld(new VisionAlignCommand(drivetrain));//
    new JoystickButton(driverController, 2).whileHeld(new EjectorCommand(ejectorSubsystem));//
    new JoystickButton(driverController, 2).whileHeld(new SpindexerSpinCommand(spindexerSubsystem));//
    new AxisTrigger(driverController, 3).whenPressed(new ShootRPMCommand2(shooterSubsystem));
    new AxisTrigger(driverController, 3).whenPressed(new SpindexerGoToMagnetCommand(spindexerSubsystem));//
    new AxisTrigger(driverController, 2).whenPressed(new ShootRPMStopCommand(shooterSubsystem));

    
    //new JoystickButton(operatorController, 1).whenHeld(new ShootRPMCommand(shooterSubsystem));//
    //new JoystickButton(operatorController, 1).whenHeld(new ShootCommand(shooterSubsystem));//
    //new JoystickButton(operatorController, 1).whenPressed(new SpindexerGoToMagnetCommand(spindexerSubsystem));//
    new JoystickButton(operatorController, 2).whenPressed(new SpindexerSpeedUpCommand(spindexerSubsystem));//
    new JoystickButton(operatorController, 3).whileHeld(new SpindexerSpinCommand(spindexerSubsystem));//
    new JoystickButton(operatorController, 4).whileHeld(new SpindexerReverseSpinCommand(spindexerSubsystem));//
    new JoystickButton(operatorController, 7).whenHeld(new HoodUpManualCommand(hoodSubsystem));//
    new JoystickButton(operatorController, 8).whenHeld(new HoodDownManualCommand(hoodSubsystem));//
    new JoystickButton(operatorController, 9).whenPressed(new HoodUpCommand(hoodSubsystem));//
    new JoystickButton(operatorController, 10).whenPressed(new HoodDownCommand(hoodSubsystem));//

    new AxisTrigger(operatorController, 2).whileHeld(new HarvestorIntakeCommand(harvestorSubsystem));//
    new AxisTrigger(operatorController, 3).whenPressed(new HarvestorIntakeStop(harvestorSubsystem));//
    new AxisTrigger(operatorController, 2).whenPressed(new HarvestorInCommand(harvestorSubsystem));//
    new AxisTrigger(operatorController, 3).whenPressed(new HarvestorOutCommand(harvestorSubsystem));//
    new JoystickButton(operatorController, 5).whenHeld(new HarvestorOuttakeCommand(harvestorSubsystem));//
    new JoystickButton(operatorController, 6).whenHeld(new HarvestorStopCommand(harvestorSubsystem));
    configureButtonBindings();
  }


  public XboxController getDriverController() {
    return driverController;
  }

  public void setRumble(){
    driverController.setRumble(RumbleType.kLeftRumble, 1);
    driverController.setRumble(RumbleType.kRightRumble, 1);
  }

  public void offRumble(){
    driverController.setRumble(RumbleType.kLeftRumble, 0);
    driverController.setRumble(RumbleType.kRightRumble, 0);
  }

  /**
   * 
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  
  private void configureButtonBindings() {

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
