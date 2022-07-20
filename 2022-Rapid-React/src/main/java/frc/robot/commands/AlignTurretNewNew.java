// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.Limelight;

public class AlignTurretNewNew extends CommandBase {

  TurretSubsystem m_turretSubsystem;

  /** Creates a new AlignTurretNewNew. */
  public AlignTurretNewNew(TurretSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turretSubsystem = subsystem; 
    addRequirements(m_turretSubsystem); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(Limelight.getTx())  > 5 && Limelight.getTv()) {
      // turret not aligned AND target exists
      m_turretSubsystem.runTurret(0.05 * (Limelight.getTx() / Math.abs(Limelight.getTx())));
    } //else {
    //   finished = true;
    // }
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
