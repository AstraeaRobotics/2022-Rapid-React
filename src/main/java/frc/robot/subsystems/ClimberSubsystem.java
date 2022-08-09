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
  private CANSparkMax climber_motor = new CANSparkMax(50, MotorType.kBrushless); // Magik Number Port
  private DigitalInput limit_switch = new DigitalInput(50); // Magik Number Port

  public ClimberSubsystem() {

  }

  public void calibrate() {
    climber_motor.setSoftLimit(SoftLimitDirection.kForward, Climber.kUpperLimit);
  }

  public void setSpeed(double speed) {
    climber_motor.set(speed);
  }

  public boolean isFullyRetracted() {
    return limit_switch.get();
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
}