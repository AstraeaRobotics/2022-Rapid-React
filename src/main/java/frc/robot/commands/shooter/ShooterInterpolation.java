// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShooterSpeeds;
import frc.robot.util.Limelight;
import frc.robot.util.Interpolation;

public class ShooterInterpolation extends CommandBase {
  /** Creates a new ShooterInterpolation. */
  ShooterSubsystem m_shooterSubsystem;

  Interpolation m_flywheelInterpolation = new Interpolation(null);
  Interpolation m_feederInterpolation = new Interpolation(null);

  public ShooterInterpolation(ShooterSubsystem shootersubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootersubsystem);
    m_shooterSubsystem = shootersubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setFeederSetpoint(ShooterSpeeds.getSpeedPercent(0.0));
    m_shooterSubsystem.setFeederSetpoint(ShooterSpeeds.getSpeedPercent(0.0));

    m_feederInterpolation.addPoint(14.1, 55);
    m_feederInterpolation.addPoint(22.6, 60);
    m_feederInterpolation.addPoint(7.4, 60);

    m_flywheelInterpolation.addPoint(14.1, 55);
    m_flywheelInterpolation.addPoint(22.6, 30);
    m_flywheelInterpolation.addPoint(7.4, 70);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Get the Y-Offset from the Limelight
    double yOffset = Limelight.getTy();

    //Get the interpolation from the UTIL class
    double flywheelSpeed = m_flywheelInterpolation.interpolate(yOffset)*100;
    double feederSpeed = m_feederInterpolation.interpolate(yOffset)*100;

    //Run motors
    m_shooterSubsystem.setFlywheelSetpoint(ShooterSpeeds.getSpeedPercent(flywheelSpeed));
    m_shooterSubsystem.setFeederSetpoint(ShooterSpeeds.getSpeedPercent(feederSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
