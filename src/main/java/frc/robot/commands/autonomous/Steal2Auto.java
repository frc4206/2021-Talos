package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Steal2Auto extends SequentialCommandGroup {

  public Steal2Auto(DrivetrainSubsystem drive) {

    addCommands(
        //new FollowPathCommand(drive, "Start line towards trench and shoot 3")
        /*new FollowPathCommand(drive, "Start line to Steal 2").deadlineWith(
            // Spin intake wheels
            new RunCommand(intake::intake),
            // Spin up shooter
            //new RunCommand(() -> shooter.run(ShooterDistances.BEHIND_LINE), shooter)),
        // Wait
        new WaitCommand(0.5).deadlineWith(
            // While keeping shooter running
            //new RunCommand(() -> shooter.run(ShooterDistances.BEHIND_LINE), shooter),
            // While keeping intake running
            new RunCommand(intake::intake)),
        // Fold up intake
        new InstantCommand(intake::retract),
        // Stop intake wheels
        new InstantCommand(intake::stop),
        // steal 2 and aim
        new FollowPathCommand(drive, "Steal 2 to Aim").deadlineWith(
            // Spin up shooter
            //new RunCommand(() -> shooter.run(ShooterDistances.BEHIND_LINE), shooter)),
        // Pew pew
        new AutonomousAlignShootCommand(drive, shooter, indexer)*/);

  }

}