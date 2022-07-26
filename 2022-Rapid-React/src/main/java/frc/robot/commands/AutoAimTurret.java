// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.Limelight;
import frc.robot.util.Logger;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

public class AutoAimTurret extends CommandBase {

  TurretSubsystem m_turretSubsystem;
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("aim");

  public AutoAimTurret(TurretSubsystem subsystem) {
    m_turretSubsystem = subsystem;
    addRequirements(m_turretSubsystem);
  }

  @Override
  public void execute() {
    if (Math.abs(Limelight.getTx()) > 5) {
      Logger.logWithNetworkTable(table, "aiming", true);
      m_turretSubsystem.runTurret(0.05 * (Limelight.getTx() / Math.abs(Limelight.getTx())));
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_turretSubsystem.runTurret(0.0);
    Logger.logWithNetworkTable(table, "aiming", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
