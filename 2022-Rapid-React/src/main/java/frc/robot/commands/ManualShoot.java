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
  private final DoubleSupplier ShootSpeed;


  public ManualShoot(ShooterSubsystem shooterSubsystem, double flywheelSpeed, double feederSpeed, DoubleSupplier ShootSpeed) {
    addRequirements(shooterSubsystem);
    this.m_shooterSubsystem = shooterSubsystem;
    this.flywheelSpeed = flywheelSpeed;
    this.feederSpeed = feederSpeed;
    this.ShootSpeed = ShootSpeed;
  }

  @Override
  public void execute() {
    double speeds = ShootSpeed.getAsDouble() * 100;
    m_shooterSubsystem.setFlywheelSetpoint(ShooterSpeeds.getSpeedPercent(speeds));
    m_shooterSubsystem.setFeederSetpoint(ShooterSpeeds.getSpeedPercent(speeds));
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
