/********************************************************************************
*                                                                               *
*   Copyright (c) Astraea Robotics, FIRST, and other WPILib contributors        *
*                                                                               *
*   Open Source Software; you can modify and/or share it under the terms of     *
*   the license file in the root directory of this project.                     *
*                                                                               *
********************************************************************************/
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
    if (Math.abs(Limelight.getTx()) < TurretConstants.kVisionThreshold) {
      m_turretSubsystem.runTurret(0);
      return;
    }
    m_turretSubsystem.runTurret(m_turretSpeed * Math.signum(Limelight.getTx()));
  }
}
