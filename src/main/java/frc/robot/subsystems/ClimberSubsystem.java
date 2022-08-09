// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  private CANSparkMax m_climber_motor = new CANSparkMax(Climber.CLIMBER_MOTOR_PORT, MotorType.kBrushless);
  private DigitalInput m_limit_switch = new DigitalInput(Climber.LIMIT_SWITCH_PORT);
  public double m_climbSpeed = Climber.kElevatorSpeed;

  public ClimberSubsystem() {

  }

  public boolean isFullyRetracted() {
    return m_limit_switch.get();
  }

  @Override
  public void periodic() {
    // Need to Fix
    // if (!isFullyRetracted()) {
    // setSpeed(-Climber.kElevateUpSpeed);
    // } else {
    // setSpeed(0);
    // }
  }

  public void setSpeed(double speed) {
    m_climber_motor.set(speed);
  }

  public void climb() {
    m_climber_motor.setSoftLimit(SoftLimitDirection.kForward, Climber.kUpperLimit);
    m_climber_motor.set(Climber.kElevatorSpeed);
  }

  public void descend() {
    m_climber_motor.setSoftLimit(SoftLimitDirection.kReverse, Climber.kLowerLimit);
    m_climber_motor.set(Climber.kElevatorSpeed);
  }

  public void stop() {
    m_climber_motor.set(0);
  }
}