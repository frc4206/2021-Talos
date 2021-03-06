// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HarvestorSubsystem;

public class HarvestorInCommand extends CommandBase {
  private final HarvestorSubsystem m_harvestor;
  public HarvestorInCommand(HarvestorSubsystem harvestor) {
    m_harvestor = harvestor;
    addRequirements(harvestor);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_harvestor.solonoidIn();
    m_harvestor.groundHarvestorStop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
