/********************************************************************************
 * * Copyright (c) Astraea Robotics, FIRST, and other WPILib contributors * * Open Source Software;
 * you can modify and/or share it under the terms of * the license file in the root directory of
 * this project. * *
 ********************************************************************************/
package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class ManualTurret extends CommandBase {

  TurretSubsystem m_turretSubsystem;
  double speed;

  public ManualTurret(TurretSubsystem subsystem, double speed) {
    m_turretSubsystem = subsystem;
    this.speed = speed;
    addRequirements(m_turretSubsystem);
  }

  @Override
  public void execute() {
    m_turretSubsystem.runTurret(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_turretSubsystem.runTurret(0);
  }
}
