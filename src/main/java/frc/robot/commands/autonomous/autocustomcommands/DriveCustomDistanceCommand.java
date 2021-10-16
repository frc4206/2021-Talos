// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.autocustomcommands;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCustomDistanceCommand extends CommandBase {
  private final DrivetrainSubsystem m_driveSubsystem;
  private double endTime;
  private final double m_forward;
  private final double m_strafe;
  private final double m_rotation;
  private final double m_endtime;



  public DriveCustomDistanceCommand(DrivetrainSubsystem driveSubsystem, double forward,
  double strafe,  double rotation, double time ) {
      m_driveSubsystem = driveSubsystem;
      m_forward = forward;
      m_rotation = rotation;
      m_strafe = strafe;
      m_endtime = time;
      addRequirements(m_driveSubsystem);
  }

  @Override
  public void initialize() {
      endTime = Timer.getFPGATimestamp() + m_endtime;
  }

  @Override
  public void execute() {
      m_driveSubsystem.drive(new Vector2(m_forward, m_strafe), m_rotation, false);
  }

  @Override
  public boolean isFinished() {
      return Timer.getFPGATimestamp() > endTime;
  }
}