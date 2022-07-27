// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.Limelight;

public class AutoAimTurret extends CommandBase {

  TurretSubsystem m_turretSubsystem;
  double m_turretSpeed;

  public AutoAimTurret(TurretSubsystem subsystem, double turretSpeed) {
    m_turretSubsystem = subsystem;
    m_turretSpeed = turretSpeed;
    addRequirements(m_turretSubsystem);
  }

  @Override
  public void execute() {
    if (Math.abs(Limelight.getTx()) > TurretConstants.kVisionThreshold)
      return;
    m_turretSubsystem.runTurret(m_turretSpeed * Math.signum(Limelight.getTx()));
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
