// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class SpindexerSubsystem extends SubsystemBase {

  //this creates our motor and our mag sensor we will use
  TalonSRX spinningMotor = new TalonSRX(RobotMap.CANIDs.spindexerSpinningMotor);

  public DigitalInput magSensor = new DigitalInput(RobotMap.CANIDs.magSensorPortSpindexer);
  private Encoder RevSpinBoreEnc = new Encoder(RobotMap.CANIDs.revSpinBoreEnc_DIO_Blue, RobotMap.CANIDs.revSpinBoreEnc_DIO_Yellow);
  //private double spinDirection = 1.0;
  //private int execCounter = 0;

  //calls the mag sensor to see if it has a target or not
  public boolean getMagSensor(){
    return magSensor.get();
  }

  //these commands all set the spindexer to move at a certain speed
  public void SpindexerSpin(){
    spinningMotor.set(ControlMode.PercentOutput, RobotMap.MotorPowerValues.spindexerSpinPower);
  }

  public void SpindexerSpinreverse(){
    spinningMotor.set(ControlMode.PercentOutput, RobotMap.MotorPowerValues.spindexerSpinReversePower);
  }

  public void SpindexerStop(){
    spinningMotor.set(ControlMode.PercentOutput, 0);
  }

  public void SpindexerConstantSpin(){
    /*execCounter = execCounter + 1;
    if (Math.abs(RevSpinBoreEnc.getRate()) > RobotMap.SpindexerValues.spindexerJamVelocityCutoff) {
        if (spinDirection == -1.0) {
            if (execCounter > RobotMap.SpindexerValues.spindexerWaitForSpeed) {
                execCounter = 0;
                spinDirection = 1.0;
            }
        }
    } else {
        if (execCounter > RobotMap.SpindexerValues.spindexerWaitForSpeed) {
            execCounter = 0;
            spinDirection = spinDirection * -1.0;
        }
    
      }*/
    spinningMotor.set(ControlMode.PercentOutput, RobotMap.MotorPowerValues.spindexerSpinConstantPower);
  }

  public void SpindexerAceleratedConstantSpin(){
    spinningMotor.set(ControlMode.PercentOutput, RobotMap.MotorPowerValues.spindexerSpinAceleratedConstantPower);
  }

  public double getSpindexerVelocity() {
    return RevSpinBoreEnc.getRate();
}

public SpindexerSubsystem() {}

  @Override
  public void periodic() {
    //readout to see if we even are getting values for the mag sensor
    SmartDashboard.putString("Magnetic Sensor", getMagSensor()+"");
    SmartDashboard.putString("Spindexer Velocity", getSpindexerVelocity()+"");
  }
}
