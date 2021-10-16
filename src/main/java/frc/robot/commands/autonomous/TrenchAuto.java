package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomous.autocustomcommands.*;
import frc.robot.commands.HarvestorOutCommand;
import frc.robot.commands.HarvestorStopCommand;
import frc.robot.commands.ShootRPMCommand;
import frc.robot.commands.ShootRPMLineCommand;
import frc.robot.commands.SpindexerGoToMagnetCommand;
import frc.robot.commands.VisionAlignCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EjectorSubsystem;
import frc.robot.subsystems.HarvestorSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

public class TrenchAuto extends SequentialCommandGroup {

    public TrenchAuto(DrivetrainSubsystem drive, ShooterSubsystem shooter, SpindexerSubsystem spindexer, HarvestorSubsystem harvestor,
            EjectorSubsystem ejector, HoodSubsystem hood) {
        addRequirements(shooter);

        addCommands(
            //the values entered are forward, then strafe, then rotation, then time(seconds) 

            //start up the shooter and ready the magazine
            new ParallelCommandGroup(
                new ShootRPMLineCommand(shooter).withTimeout(2.5),//2 with noo shoot
                new SpindexerGoToMagnetCommand(spindexer)
            ),

            //shoot at the target
            new AutonomousShootLineCommand(drive, shooter, spindexer, ejector).withTimeout(3),

            //spin 90 degress and wait to lower shaking
            new DriveCustomDistanceCommand(drive, 0, 0, 0.04, 0.7),                 
            new WaitCommand(0.3),

            //drive towards the wall and the three balls
            new DriveCustomDistanceCommand(drive, 0.5, 0.5, -0.00, 1),

            //drive into the wall for aligning purposes
            new DriveCustomDistanceCommand(drive, 0, 0.2, 0, 1.3),

            //drive away from the wall and lower the harvestor for feeding
            new ParallelCommandGroup(
                new DriveCustomDistanceCommand(drive, 0, -0.2, 0, 0.5),
                new HarvestorOutCommand(harvestor).withTimeout(0.1)
            ),
            new WaitCommand(0.1),

            //drive towards and into the balls while running the harvestor
            new ParallelCommandGroup(
                new DriveCustomDistanceCommand(drive, 0.5, -0.01, 0, 1.9),                
                new SpindexerConstantSpinCommand(spindexer).withTimeout(1.5)
            ),

            //drive towards the target while getting ready to shoot
            new ParallelCommandGroup(                
                new DriveCustomDistanceCommand(drive, -0.71, -0.2, 0, 0.5),//1
                new ShootRPMCommand(shooter).withTimeout(0.5),
                new SpindexerGoToMagnetCommand(spindexer)
            ),

            //spin to get in the general area of the target
            new ParallelCommandGroup(
                new DriveCustomDistanceCommand(drive, 0, 0, 0.05, 0.6),
                new ShootRPMCommand(shooter).withTimeout(0.6)
            ),

            
            //align command
            new ParallelCommandGroup(
                new VisionAlignCommand(drive).withTimeout(0.78),
                new ShootRPMCommand(shooter).withTimeout(0.78)
            ),

            //shoot the three balls you just picked up
            new ParallelCommandGroup(
                new AutonomousAlignShootCommand(drive, shooter, spindexer, ejector).withTimeout(2),
                new HarvestorStopCommand(harvestor)
            )
            //now drive for real
        );
    }

}