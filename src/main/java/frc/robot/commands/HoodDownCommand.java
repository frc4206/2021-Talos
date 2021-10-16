// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class HoodDownCommand extends CommandBase {
  private final HoodSubsystem m_hood;
  private boolean magnetdetected;
  private boolean commandisfinished;
    public HoodDownCommand(HoodSubsystem hood) {
      m_hood = hood;
      addRequirements(hood);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      if(m_hood.getHoodPosition() == false){
        commandisfinished = true;
      }
      else{
        magnetdetected = m_hood.getMagSensor();
        commandisfinished = false;
      }
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if(magnetdetected == m_hood.getMagSensor()){
        m_hood.servoDownward();
      }
      else {
  
          if(m_hood.getMagSensor() == false){
              m_hood.servoStop();
              commandisfinished = true;
              m_hood.setHoodPosition(false);
          }
  
          else{
              m_hood.servoDownward();
              magnetdetected = m_hood.getMagSensor();
          }
  
      }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_hood.servoStop();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return commandisfinished;
    }
  }
