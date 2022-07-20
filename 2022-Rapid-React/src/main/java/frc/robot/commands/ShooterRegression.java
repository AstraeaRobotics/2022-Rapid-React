// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.Limelight;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Regression;
import frc.robot.util.ShooterSpeeds;

public class ShooterRegression extends CommandBase {

  private final ShooterSubsystem m_shooterSubsystem;

  public ShooterRegression(ShooterSubsystem shooterSubsystem) {
    addRequirements(shooterSubsystem);
    this.m_shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.setFeederSetpoint(ShooterSpeeds.getSpeedPercent(0.0));
    m_shooterSubsystem.setFeederSetpoint(ShooterSpeeds.getSpeedPercent(0.0));
  }

  @Override
  public void execute() {

    double flywheelSpeed = Regression.binomialRegression( 
      Shooter.kFlywheelA, 
      Shooter.kFlywheelB, 
      Shooter.kFeederC,
      Limelight.getTy()
    );

    double feederSpeed = Regression.binomialRegression(
      Shooter.kFeederA, 
      Shooter.kFeederB, 
      Shooter.kFeederC,
      Limelight.getTy()
    );

    m_shooterSubsystem.setFlywheelSetpoint(flywheelSpeed);
    m_shooterSubsystem.setFeederSetpoint(feederSpeed);
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
