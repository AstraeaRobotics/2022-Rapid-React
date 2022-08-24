// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class PureFeedForwardControl extends CommandBase {
  ShooterSubsystem m_ShooterSubsystem;
  double flywheelSetpoint;
  double feederSetpoint;

  SimpleMotorFeedforward flywheelFeedForward = new SimpleMotorFeedforward(0, 0.0018, 0.0029);
  SimpleMotorFeedforward feederFeedForward = new SimpleMotorFeedforward(0, 0.18, 0.29);

  /** Creates a new ShooterPIDCommand. */
  public PureFeedForwardControl(ShooterSubsystem shooterSubsystem, double flywheelSetpoint, double feederSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
    m_ShooterSubsystem = shooterSubsystem;
    this.flywheelSetpoint = flywheelSetpoint;
    this.feederSetpoint = feederSetpoint;

    m_ShooterSubsystem.setFlywheelSetpoint(flywheelSetpoint);
    m_ShooterSubsystem.setFeederSetpoint(feederSetpoint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.setFlyWheelInputVoltage(flywheelFeedForward.calculate(flywheelSetpoint));
    m_ShooterSubsystem.setFeederInputVoltage(feederFeedForward.calculate(m_ShooterSubsystem.getFeederRPM(), feederSetpoint, 0.020));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.setFlyWheelInputVoltage(0);
    m_ShooterSubsystem.setFeederInputVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
