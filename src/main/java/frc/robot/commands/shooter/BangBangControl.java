// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class BangBangControl extends CommandBase {
  private ShooterSubsystem m_shooterSubsystem;

  public BangBangControl(ShooterSubsystem m_shooterSubsystem, double flywheelSetpoint, double feederSetpoint) {
    addRequirements(m_shooterSubsystem);
    this.m_shooterSubsystem = m_shooterSubsystem;
    m_shooterSubsystem.setFlywheelSetpoint(flywheelSetpoint);
    m_shooterSubsystem.setFeederSetpoint(feederSetpoint);
  }

  @Override
  public void execute() {
    if (m_shooterSubsystem.getFlywheelRPM() >= m_shooterSubsystem.getFlywheelSetpoint()) {
      m_shooterSubsystem.setFlyWheelInputVoltage(0);
    } else {
      m_shooterSubsystem.setFlyWheelInputVoltage(2);
    }
    
    if (m_shooterSubsystem.getFeederRPM() >= m_shooterSubsystem.getFeederSetPoint()) {
      m_shooterSubsystem.setFeederInputVoltage(0);
    } else {
      m_shooterSubsystem.setFeederInputVoltage(2);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setFlyWheelInputVoltage(0);
    m_shooterSubsystem.setFeederInputVoltage(0);
  }
}
