// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.Limelight;

public class AutoAimTurret extends CommandBase {

  TurretSubsystem m_turretSubsystem;

  public AutoAimTurret(TurretSubsystem subsystem) {
    m_turretSubsystem = subsystem;
    addRequirements(m_turretSubsystem);
  }

  @Override
  public void execute() {
    if (Math.abs(Limelight.getTx()) > 5) {
      m_turretSubsystem.runTurret(0.05 * (Limelight.getTx() / Math.abs(Limelight.getTx())));
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_turretSubsystem.runTurret(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
