// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootRPMCommand2 extends CommandBase {
  private final ShooterSubsystem m_shooter;
  

  public ShootRPMCommand2(ShooterSubsystem shooter) {
    m_shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_shooter.configClosedLoop();
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double convertedVelocity;

    if(Robot.getRobotContainer().getDriverController().getRawButton(6))
    {
      convertedVelocity = ((((RobotMap.MotorPowerValues.shooterRPMValue+100)*10*60)/2048)*(30/18));
    }
    else if(Robot.getRobotContainer().getDriverController().getRawButton(5)) 
    {
      convertedVelocity = ((((RobotMap.MotorPowerValues.shooterRPMValue-100)*10*60)/2048)*(30/18));
    }
    else{
      convertedVelocity = ((((RobotMap.MotorPowerValues.shooterRPMValue)*10*60)/2048)*(30/18));
    }

    m_shooter.setShootSpeed(convertedVelocity);



    if(m_shooter.getRPM()>RobotMap.MotorPowerValues.shooterRPMValue-100){
      m_shooter.setShooterSpeed(true);
      Robot.getRobotContainer().setRumble();
    }
    else{
      Robot.getRobotContainer().offRumble();
      m_shooter.setShooterSpeed(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().offRumble();
    m_shooter.setShooterSpeed(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
/*

    double convertedVelocity = ((((RobotMap.MotorPowerValues.shooterRPMValue)*10*60)/2048)*(30/18));
    //to change velo change this #   ^

    m_shooter.setShootSpeed(convertedVelocity);

    if(m_shooter.getRPM()>RobotMap.MotorPowerValues.shooterRPMValue-50){
      m_shooter.setShooterSpeed(true);
    }
    else{
      m_shooter.setShooterSpeed(false);
    }
    --------------------------------------------------------------------------------------------------------------------------------------
    
        if(Robot.getRobotContainer().getDriverController().getRawButton(3))
        {
            double convertedVelocity = ((((RobotMap.MotorPowerValues.shooterRPMValue+100)*10*60)/2048)*(30/18));
        } else { 
            double convertedVelocity = ((((RobotMap.MotorPowerValues.shooterRPMValue+100)*10*60)/2048)*(30/18));

        }

                if(Robot.getRobotContainer().getDriverController().getRawButton(3))
        {
            double convertedVelocity = ((((RobotMap.MotorPowerValues.shooterRPMValue-100)*10*60)/2048)*(30/18));
        } else { 
            double convertedVelocity = ((((RobotMap.MotorPowerValues.shooterRPMValue+100)*10*60)/2048)*(30/18));

        }
*/