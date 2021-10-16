package frc.robot.commands.autonomous.autocustomcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.EjectorCommand;
import frc.robot.commands.ShootRPMCommand;
import frc.robot.commands.SpindexerSpinCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EjectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

/**
 * In parallel spin up the shooter wheels and vision  (timeout 1.5). Once
 * both are done, feeds balls into shooter for 3 seconds
 */
public class AutonomousShootCommand extends SequentialCommandGroup {
    public AutonomousShootCommand(DrivetrainSubsystem drive, ShooterSubsystem shooter, SpindexerSubsystem spindexer, EjectorSubsystem ejector) {

        addCommands(sequence(
                new ParallelCommandGroup(
                        new ShootRPMCommand(shooter).withTimeout(2.5),
                        new SpindexerSpinCommand(spindexer).withTimeout(2.5),
                        new EjectorCommand(ejector).withTimeout(2.5)
                )        
                
        ));
        }
}
