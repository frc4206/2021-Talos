// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SpindexerSubsystem;

public class SpindexerGoToMagnetCommand extends CommandBase {
  private final SpindexerSubsystem m_spindexer;
  private boolean commandIsFinished;
  public SpindexerGoToMagnetCommand(SpindexerSubsystem spindexer) {
    m_spindexer = spindexer;
    addRequirements(spindexer);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    commandIsFinished = false;
  }

  @Override
  public void execute(){
      if (m_spindexer.getMagSensor() == true)
       {  m_spindexer.SpindexerConstantSpin(); }
      else {
          commandIsFinished = true;
          end(false); }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_spindexer.SpindexerStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandIsFinished;
  }
}
