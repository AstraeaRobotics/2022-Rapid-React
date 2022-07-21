// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShooterSpeeds;

public class ManualShoot extends CommandBase {

  private ShooterSubsystem m_shooterSubsystem;

  private final double flywheelSpeed;
  private final double feederSpeed;

  public ManualShoot(ShooterSubsystem shooterSubsystem, double flywheelSpeed, double feederSpeed) {
    addRequirements(shooterSubsystem);
    this.m_shooterSubsystem = shooterSubsystem;
    this.flywheelSpeed = flywheelSpeed;
    this.feederSpeed = feederSpeed;
  }

  @Override
  public void execute() {
    m_shooterSubsystem.setFlywheelSetpoint(ShooterSpeeds.getSpeedPercent(50));
    m_shooterSubsystem.setFeederSetpoint(ShooterSpeeds.getSpeedPercent(80));
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
