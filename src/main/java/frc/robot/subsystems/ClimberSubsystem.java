// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {

  private CANSparkMax m_climberMotor;
  private DigitalInput m_limitSwitch;
  public double m_climbSpeed;
  private final RelativeEncoder m_encoder;
  boolean toggleLimit;

  public ClimberSubsystem() {
    m_climberMotor =
        new CANSparkMax(Climber.kClimberMotor_Port, MotorType.kBrushless);
    m_limitSwitch = new DigitalInput(1);
    m_encoder = m_climberMotor.getEncoder();
    m_encoder.setPosition(0);
    // setSoftLimits();
    disableSoftLimits();
    m_climbSpeed = Climber.kElevatorSpeed;
    m_climberMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Height", m_encoder.getPosition());
    SmartDashboard.putBoolean("Limit Switch", m_limitSwitch.get());
  }

  public boolean fullyRetracted() {
    return m_limitSwitch.get();
  }

  public void disableSoftLimits() {
    m_climberMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_climberMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  public void setSoftLimits() {
    m_climberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_climberMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_climberMotor.setSoftLimit(SoftLimitDirection.kForward, Climber.kUpperLimit); // 15
    m_climberMotor.setSoftLimit(SoftLimitDirection.kReverse, Climber.kLowerLimit); // 0
  }

  public void setSpeed(double speed) {
    m_climberMotor.set(speed);
  }

  public void reset() {
    m_encoder.setPosition(0);
  }

  public void stop() {
    setSpeed(0);
  }
}
