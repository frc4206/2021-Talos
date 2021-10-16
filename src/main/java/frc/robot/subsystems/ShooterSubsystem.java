// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSubsystem extends SubsystemBase {

  //These are the two falcons we will use for the shooter
  WPI_TalonFX Shooter1 = new WPI_TalonFX(RobotMap.CANIDs.shooter1);
  WPI_TalonFX Shooter2 = new WPI_TalonFX(RobotMap.CANIDs.shooter2);

  //creates a bolean so that we can setup a smartdashboard rpm
  private boolean shooterAtSpeed = false;

  private double velo = 0.0;

  public boolean getShooterSpeed(){
    return shooterAtSpeed;
  }

  public void setShooterSpeed(boolean shooterAtSpeedSet){
    shooterAtSpeed = shooterAtSpeedSet;
  }
  

  public void ShooterHoodSubsystem() {
    //inverts a motor so they do not run against each other
    //the motor follows it so you don't have to command both
    Shooter1.configFactoryDefault();
    Shooter2.configFactoryDefault();

    //About: configure the motors 
    configureMotors();
    configClosedLoop();

  }


  private void configureMotors(){
    Shooter1.setInverted(false);
    Shooter1.configOpenloopRamp(2.0, 0);
    Shooter1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,0);
    Shooter1.setSensorPhase(true); 
    
    Shooter2.setInverted(true);
    Shooter2.configOpenloopRamp(2.0, 0);
    Shooter2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,0);
    Shooter2.setSensorPhase(true); 
  }


  public void configClosedLoop(){
    Shooter1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    Shooter2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    Shooter1.configClosedloopRamp(0.5);
    Shooter2.configClosedloopRamp(0.5);
    Shooter1.configAllowableClosedloopError(0, 10);
    Shooter2.configAllowableClosedloopError(0, 10);

    //About: how to calculate kF https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html#calculating-velocity-feed-forward-gain-kf
    //Shooter1.config_kF(0, getShooterkF(6600*Constants.ShooterConstants.kRPMtoTicks));
    Shooter1.config_kP(0, 0.002);
    Shooter1.config_kI(0, 0);
    Shooter1.config_kD(0, 0);

    //Shooter2.config_kF(0, getShooterkF(6600*Constants.ShooterConstants.kRPMtoTicks));
    Shooter2.config_kP(0, 0.002);
    Shooter2.config_kI(0, 0);
    Shooter2.config_kD(0, 0);    
  }

  public void ShootCommand(){
    Shooter1.set(ControlMode.PercentOutput, RobotMap.MotorPowerValues.shooterWheelPercentOuput);
    Shooter2.set(ControlMode.PercentOutput, -RobotMap.MotorPowerValues.shooterWheelPercentOuput);
  }

  public void ShootCommandStop(){
    Shooter1.set(ControlMode.PercentOutput, 0);
    Shooter2.set(ControlMode.PercentOutput, 0);
  }


  public void setPowerMode(){
    Shooter1.configNominalOutputForward(0.0, 0);
    Shooter1.configNominalOutputReverse(0.0, 0);
    Shooter1.setNeutralMode(NeutralMode.Coast);

    Shooter2.configNominalOutputForward(0.0, 0);
    Shooter2.configNominalOutputReverse(0.0, 0);
    Shooter2.setNeutralMode(NeutralMode.Coast);

  }

  //get our rpm for the shooter
  public double getRPM(){
    return ((((Shooter1.getSelectedSensorVelocity())*10*60)/2048)*(30/18))*(6.68/4);
  }


  public double HoodKF(){
    double Velo = (1024/36);
    double percentOut = 100;
    double Kf = (percentOut*1023)/Velo;

    return Kf; 
  }


  public void setShootSpeed(double velocity){
    velo = velocity;
    Shooter1.set(TalonFXControlMode.Velocity, velo);
    Shooter2.set(TalonFXControlMode.Velocity, -velo);
  }


  public void shooterBolean(){
    if(getRPM() >= 6500){
      shooterAtSpeed = true;
    }
    else{
      shooterAtSpeed = false;
    }
  }


  public void setPower(double power){
    setPowerMode();
    Shooter1.set(ControlMode.PercentOutput, power);
    Shooter2.set(ControlMode.PercentOutput, -power);
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putString("Shooter RPM", getRPM()+"");
    SmartDashboard.putBoolean("Shooter at Speed", shooterAtSpeed);
    SmartDashboard.putString("shooterCodeRPM", Shooter1.getSelectedSensorVelocity()+"");
  }
}
