// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class CenterTurret extends CommandBase {
  TurretSubsystem turret;

  public CenterTurret(TurretSubsystem turret) {
    this.turret = turret;
  }

  @Override
  public void execute() {
    if (Math.abs(turret.getCurrentPosition()) > 1) {
      turret.runTurret(-Math.signum(turret.getCurrentPosition()) * .2);
      System.out.println("centering");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.runTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
