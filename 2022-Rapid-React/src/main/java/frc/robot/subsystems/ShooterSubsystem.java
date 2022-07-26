// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;
import frc.robot.util.ShooterSpeeds;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

public class ShooterSubsystem extends SubsystemBase {

  TalonFX feeder = new TalonFX(10);
  TalonFX flywheel = new TalonFX(11);

  ShooterSpeeds m_shooterSpeeds = new ShooterSpeeds(0, 0);

  public ShooterSubsystem() {
    feeder.setNeutralMode(NeutralMode.Coast);
    flywheel.setNeutralMode(NeutralMode.Coast);

    feeder.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Shooter.kPIDLoopIDx, Shooter.kTimeoutMs);
    feeder.config_kF(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkF, Shooter.kTimeoutMs);
    feeder.config_kP(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkP, Shooter.kTimeoutMs);
    feeder.config_kI(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkI, Shooter.kTimeoutMs);
    feeder.config_kD(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkD, Shooter.kTimeoutMs);

    flywheel.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Shooter.kPIDLoopIDx, Shooter.kTimeoutMs);
    flywheel.config_kF(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkF, Shooter.kTimeoutMs);
    flywheel.config_kP(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkP, Shooter.kTimeoutMs);
    flywheel.config_kI(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkI, Shooter.kTimeoutMs);
    flywheel.config_kD(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkD, Shooter.kTimeoutMs);
  }

  public void stopMotors() {
    m_shooterSpeeds.setBottomVelocity(0.0);
    m_shooterSpeeds.setTopVelocity(0.0);
    flywheel.set(ControlMode.PercentOutput, 0.0);
    feeder.set(ControlMode.PercentOutput, 0.0);
  }
  
  /**
  * Changes the flywheel setpoint
  * @param speed the percent of max speed to change setpoint to (between 0 and 100)
  */
 public void setFlywheelSetpoint(double speed) {
   m_shooterSpeeds.setTopVelocity(speed);
 }

  /**
   * Changes the feeder setpoint
   * @param speed the percent of max speed to change setpoint to (between 0 and 100)
   */
  public void setFeederSetpoint(double speed) {
    m_shooterSpeeds.setBottomVelocity(speed);
  }

  private void runMotors() {
    flywheel.set(ControlMode.Velocity, m_shooterSpeeds.getTopVelocity());
    feeder.set(ControlMode.Velocity, m_shooterSpeeds.getBottomVelocity());
  }

  @Override
  public void periodic() {
    runMotors();
  }

}
