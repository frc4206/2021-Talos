// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.autocustomcommands.*;
import frc.robot.commands.HarvestorInCommand;
import frc.robot.commands.HarvestorOutCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SpindexerGoToMagnetCommand;
import frc.robot.commands.autonomous.autocustomcommands.DriveCustomDistanceCommand;
import frc.robot.commands.autonomous.autocustomcommands.DriveCustomDistanceFieldOrienedCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EjectorSubsystem;
import frc.robot.subsystems.HarvestorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrenchOffsetAuto extends SequentialCommandGroup {
  /** Creates a new TrenchOffsetAuto. */
  public TrenchOffsetAuto(DrivetrainSubsystem drive, ShooterSubsystem shooter, SpindexerSubsystem spindexer,
  HarvestorSubsystem harvestor, EjectorSubsystem ejector) {
    addRequirements(shooter);
    addCommands(

      //ready up the shooter and put the spindexer in position

      new ParallelCommandGroup(
        new ShootCommand(shooter).withTimeout(2),
        new SpindexerGoToMagnetCommand(spindexer)
      ),

      //shoot the balls with limelight tracking
      new AutonomousAlignShootCommand(drive, shooter, spindexer, ejector).withTimeout(2.5),

      //spin 90 degress and wait to lower shaking
      new DriveCustomDistanceCommand(drive, 0, 0, -0.04, 0.5),                 
      new WaitCommand(.2),

      //new HarvestorOutCommand(harvestor).withTimeout(0.001),

      //move backwards
      new DriveCustomDistanceFieldOrienedCommand(drive, -0.5, 0.03, 0, 0.9),

      //drive into the wall for aligning purposes
      new DriveCustomDistanceCommand(drive, 0, 0.2, 0, 2),

      //drive away from the wall and lower the harvestor for feeding
      new ParallelCommandGroup(
          new DriveCustomDistanceCommand(drive, 0, -0.2, 0, 0.5),
          new HarvestorOutCommand(harvestor).withTimeout(0.1)
      ),
      new WaitCommand(0.1),

      new DriveCustomDistanceCommand(drive, 0.65, -0.01, 0, 1.7),
      
      //drive towards the target while getting ready to shoot
      new ParallelCommandGroup(                
        new DriveCustomDistanceCommand(drive, -1, -0.6, 0, 1.3),
        new ShootCommand(shooter).withTimeout(1.3),
        new SpindexerGoToMagnetCommand(spindexer)
      ),
    
      //spin and continue reving up the flyweel
      new ParallelCommandGroup(
        new DriveCustomDistanceCommand(drive, 0, 0, -0.05, 0.78),
        new ShootCommand(shooter).withTimeout(0.78),
        new HarvestorInCommand(harvestor).withTimeout(0.78)
      ),
    
      //shoot the three balls you just picked up
      new AutonomousAlignShootCommand(drive, shooter, spindexer, ejector).withTimeout(2.5)
      //now drive for real
      
    );
  }

}
