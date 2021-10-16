// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class HoodSubsystem extends SubsystemBase {

  //creates the servos we will use to power the hood
  Servo testServo0 = new Servo(0);
  Servo testServo1 = new Servo(1);

  public DigitalInput magSensor = new DigitalInput(RobotMap.CANIDs.magSensorPortHood);

  boolean hoodUpDown = false;// true = up and false = down

  public boolean getHoodPosition(){
    return hoodUpDown;
  }

  public void setHoodPosition(boolean upDownParm){
      hoodUpDown = upDownParm;
  }

  public boolean getMagSensor(){
    return magSensor.get();
  }

  public void servoUpward(){
    testServo0.setRaw(1473);
    testServo1.setRaw(500);
    //set the servos to move upward
    //they stop at 0 and 1000, small diffrence to acount for mechanical stuff
    //225 works
    //1000 stops it
    //1750 almpst works too fast tho
  }

  public void servoDownward(){
    testServo0.setRaw(500);
    testServo1.setRaw(1473);
    //225 works
    //1000 stops it
    //1750 almpst works too fast tho
  }

  public void servoStop(){
    testServo0.setRaw(0);
    testServo1.setRaw(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Magnetic Sensor Hoood", getMagSensor()+"");
    SmartDashboard.putString("Hood Up (true) hood down (false)", getHoodPosition()+"");
    SmartDashboard.putBoolean("Hood Up (true) hood down (false)", getHoodPosition());
  }
}
