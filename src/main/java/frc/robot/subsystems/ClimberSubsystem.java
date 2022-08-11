// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Climber;
import frc.robot.commands.climber.Calibrate;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private CANSparkMax m_climberMotor;
  private DigitalInput m_limitSwitch;
  public double m_climbSpeed;
  private final RelativeEncoder m_encoder;
  private boolean buttonPressed;

  public ClimberSubsystem() {
    m_climberMotor = new CANSparkMax(Climber.kClimberMotor_Port, MotorType.kBrushless);
    m_limitSwitch = new DigitalInput(Climber.kLimitSwitch_Port);
    m_encoder = m_climberMotor.getEncoder();
    m_encoder.setPosition(0);
    m_climbSpeed = Climber.kElevatorSpeed;
    buttonPressed = false;
  }

  public boolean isFullyRetracted() {
    return m_limitSwitch.get();
  }

  @Override
  public void periodic() {
    if (!isFullyRetracted()) {
      m_climberMotor.set(-m_climbSpeed);
    } else {
      m_climberMotor.set(0);
    }
  }

  public boolean buttonPressed(boolean pressed) {
    buttonPressed = pressed;
    return buttonPressed;
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

  public void climb() {
    m_climberMotor.set(Climber.kElevatorSpeed);
  }

  public void descend() {
    m_climberMotor.set(-Climber.kElevatorSpeed);
  }

  public void stop() {
    m_climberMotor.set(0);
  }
}