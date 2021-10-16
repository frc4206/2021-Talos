// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class HarvestorSubsystem extends SubsystemBase {

  
  private CANSparkMax harvestorMotor = new CANSparkMax(RobotMap.CANIDs.HarvestorMotor, MotorType.kBrushless);

  private Solenoid harvestorSolenoid = new Solenoid(1);
  
  public HarvestorSubsystem(){
    harvestorMotor.setSmartCurrentLimit(50);
  }

  public void solonoidIn(){
    harvestorSolenoid.set(false);
  }

  public void solonoidOut(){
    harvestorSolenoid.set(true);
  }


  public void groundHarvestorIn(){
    harvestorMotor.set(RobotMap.MotorPowerValues.groundHarvestorIn);
  }

  public void reverseGroundHarvestor(){
    harvestorMotor.set(RobotMap.MotorPowerValues.groundHarvestorReverse);
  }

  public void groundHarvestorStop(){
    harvestorMotor.set(0);
  }

  @Override
  public void periodic() {
  }
}
