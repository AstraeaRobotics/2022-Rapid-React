// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.math.filter.LinearFilter;

public class AutonomousShooterCommand extends CommandBase {
  //linear filter
  private final LinearFilter m_filter = new LinearFilter(null, null);
  private LinearFilter highPassFilter;
  ShooterSubsystem m_shooterSubsystem;

  /** Creates a new AutonomousShooterCommand. */
  public AutonomousShooterCommand(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
    m_shooterSubsystem = shooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //highPassFilter = m_filter.highPass(timeConstant, period);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double feederVelocity = m_shooterSubsystem.getFeederSensorVelocity();
    double flywheelVelocity = m_shooterSubsystem.getFlywheelSensorVelocity();
    if (highPassFilter.calculate(feederVelocity) > 0.75) { //Magic value for now
      return true;
    }
    if (highPassFilter.calculate(flywheelVelocity) > 0.75) { //Magic value for now
      return true;
    }

    return false;
  }
}
