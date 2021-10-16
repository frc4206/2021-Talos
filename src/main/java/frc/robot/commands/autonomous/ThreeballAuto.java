// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.autocustomcommands.*;
import frc.robot.commands.ShootRPMLineCommand;
import frc.robot.commands.SpindexerGoToMagnetCommand;
import frc.robot.commands.VisionAlignCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EjectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeballAuto extends SequentialCommandGroup {
  /** Creates a new AutoAlignAndShootTest. */
  public ThreeballAuto(DrivetrainSubsystem drive, ShooterSubsystem shooter, SpindexerSubsystem spindexer,
  EjectorSubsystem ejector) {
    addRequirements(shooter);

    addCommands(
      
      //ready up the shooter and put the spindexer in position
      new ParallelCommandGroup(
        new ShootRPMLineCommand(shooter).withTimeout(3),
        new SpindexerGoToMagnetCommand(spindexer)
      ),

      new ParallelCommandGroup(
        new VisionAlignCommand(drive).withTimeout(3),
        new ShootRPMLineCommand(shooter).withTimeout(3)
      ),

      //shoot the balls with limelight tracking
      new AutonomousShootLineCommand(drive, shooter, spindexer, ejector).withTimeout(3),

      //move backwards
      new DriveCustomDistanceFieldOrienedCommand(drive, -0.5, 0, 0, 0.5)
      );
  } 
}
