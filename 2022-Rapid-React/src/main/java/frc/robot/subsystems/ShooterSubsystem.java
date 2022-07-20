// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;
import frc.robot.util.ShooterSpeeds;

import com.ctre.phoenix.motorcontrol.ControlMode; //Motor speed/control
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX; //Defines motor
import com.ctre.phoenix.motorcontrol.NeutralMode; //Motor brake/run
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShootingSubsystem. */
  TalonFX lowSrx = new TalonFX(10);
  TalonFX topSrx = new TalonFX(11);

  ShooterSpeeds m_shooterSpeeds = new ShooterSpeeds(0, 0);
  
  public ShooterSubsystem() {
    lowSrx.setNeutralMode(NeutralMode.Coast); //Sets motor to On/Coast
    topSrx.setNeutralMode(NeutralMode.Coast);
  }

  public void setMotors(double top, double bottom) {
    lowSrx.set(ControlMode.PercentOutput, top);
    topSrx.set(ControlMode.PercentOutput, bottom);
  }

  public void stopMotors() {
    lowSrx.set(ControlMode.PercentOutput, 0.0);
    topSrx.set(ControlMode.PercentOutput, 0.0);
  }

  public void setSpeedLower(double speed) {
    m_shooterSpeeds.setBottomVelocity(speed);
  }

  public void setSpeedUpper(double speed) {
    m_shooterSpeeds.setTopVelocity(speed);
  }

  private void lowSrxFPID(double speed) {    
    lowSrx.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Shooter.kPIDLoopIDx, Shooter.kTimeoutMs);
    lowSrx.config_kF(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkF, Shooter.kTimeoutMs);
		lowSrx.config_kP(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkP, Shooter.kTimeoutMs);
		lowSrx.config_kI(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkI, Shooter.kTimeoutMs);
		lowSrx.config_kD(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkD, Shooter.kTimeoutMs);
    
    //double targetVelocity_UnitsPer100ms = speed * 2000.00 * Shooter.kConversionFactor;
    //TODO: this is scuffed
    lowSrx.set(TalonFXControlMode.Velocity, ShooterSpeeds.getSpeedPercent(speed));

    SmartDashboard.putNumber("Bottom Target Velocity", ShooterSpeeds.getSpeedPercent(speed));
    SmartDashboard.putNumber("Bottom Velocity", lowSrx.getSelectedSensorVelocity());
  }

  private void topSrxFPID(double speed) {    
    topSrx.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Shooter.kPIDLoopIDx, Shooter.kTimeoutMs);
    topSrx.config_kF(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkF, Shooter.kTimeoutMs);
    topSrx.config_kP(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkP, Shooter.kTimeoutMs);
    topSrx.config_kI(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkI, Shooter.kTimeoutMs);
    topSrx.config_kD(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkD, Shooter.kTimeoutMs);

    // double targetVelocity_UnitsPer100ms = speed * 2000.000 * Shooter.kConversionFactor;
    topSrx.set(TalonFXControlMode.Velocity, ShooterSpeeds.getSpeedPercent(speed));

    SmartDashboard.putNumber("Top Target Velocity", ShooterSpeeds.getSpeedPercent(speed));
    SmartDashboard.putNumber("Top Velocity", topSrx.getSelectedSensorVelocity());
  }

  public double speedEquation(double speed) {
    var a = 1;
    var b = 1;
    var c = 0;
    return (a*Math.pow(speed, a)) + (b*speed) + c;
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    lowSrxFPID(m_shooterSpeeds.getBottomVelocity());
    topSrxFPID(m_shooterSpeeds.getTopVelocity());
  }
}
