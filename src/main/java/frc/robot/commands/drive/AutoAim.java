// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Limelight;

public class AutoAim extends CommandBase {

  DriveSubsystem m_driveSubsystem;
  double y_offset;
  private double kP = .2;

  public AutoAim(DriveSubsystem drive) {
    this.m_driveSubsystem = drive;
  }

  @Override
  public void initialize() {
    y_offset = Limelight.getTy();
  }

  @Override
  public void execute() {
    y_offset = Limelight.getTy();
    double y_adjust = kP * y_offset;
    y_adjust = MathUtil.clamp(y_adjust, .5, 1);
    SmartDashboard.putNumber("y_adjust", y_adjust);
    if(Math.abs(y_offset) > 1) {
      SmartDashboard.putBoolean("y aligned", false);
      m_driveSubsystem.tankDrive(Math.signum(y_offset) * y_adjust, Math.signum(y_offset) * y_adjust);
    } else {
      SmartDashboard.putBoolean("y aligned", true);
      m_driveSubsystem.tankDrive(0, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.tankDrive(0, 0);
  }

}
