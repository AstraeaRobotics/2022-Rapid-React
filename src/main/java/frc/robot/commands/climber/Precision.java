// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimberSubsystem;

import java.util.concurrent.ThreadPoolExecutor.DiscardOldestPolicy;

import edu.wpi.first.math.controller.PIDController;

public class Precision extends CommandBase {
  private ClimberSubsystem m_climberSubsystem;
  private double m_distance;
  private PIDController pid;

  /** Creates a new precision. */
  public Precision(ClimberSubsystem climberSubsystem, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climberSubsystem);
    m_climberSubsystem = climberSubsystem;
    m_distance = distance;

    pid = new PIDController(Climber.kP, Climber.kI, Climber.kD);
    pid.setTolerance(1, 2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberSubsystem.setSpeed(pid.calculate(m_climberSubsystem.getDistance(), m_distance));
    if (pid.atSetpoint()) {
      m_climberSubsystem.stop();
    }
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
