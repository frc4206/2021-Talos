// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EjectorSubsystem;

public class EjectorCommand extends CommandBase {
  private final EjectorSubsystem m_ejectorSubsystem;
  private double  m_speed;
  public EjectorCommand(EjectorSubsystem ejectorSubsystem) {
    m_ejectorSubsystem = ejectorSubsystem;
    addRequirements(ejectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_speed = 0.28;//0.3
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_speed = m_speed + 0.01;
    if (m_speed>0.9){
      m_speed = 0.9;
    }
    m_ejectorSubsystem.Ejector(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ejectorSubsystem.EjectorReverse();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
