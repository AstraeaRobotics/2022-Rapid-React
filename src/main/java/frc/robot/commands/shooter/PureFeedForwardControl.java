// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class PureFeedForwardControl extends CommandBase {
  ShooterSubsystem m_ShooterSubsystem;
  PIDController flywheelPID = new PIDController(0, 0, 0);
  PIDController feederPID = new PIDController(0, 0, 0);

  /** Creates a new ShooterPIDCommand. */
  public PureFeedForwardControl(ShooterSubsystem shooterSubsystem, double flywheelSetpoint, double feederSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
    m_ShooterSubsystem = shooterSubsystem;

    flywheelPID.setSetpoint(flywheelSetpoint);
    feederPID.setSetpoint(feederSetpoint);
    m_ShooterSubsystem.setFlywheelSetpoint(flywheelSetpoint);
    m_ShooterSubsystem.setFeederSetpoint(feederSetpoint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.setFlyWheelInputVoltage(flywheelPID.calculate(m_ShooterSubsystem.getFlywheelRPM()));
    m_ShooterSubsystem.setFeederInputVoltage(feederPID.calculate(m_ShooterSubsystem.getFeederRPM()));
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
