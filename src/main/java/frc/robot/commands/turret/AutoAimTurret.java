/********************************************************************************
*                                                                               *
*   Copyright (c) Astraea Robotics, FIRST, and other WPILib contributors        *
*                                                                               *
*   Open Source Software; you can modify and/or share it under the terms of     *
*   the license file in the root directory of this project.                     *
*                                                                               *
********************************************************************************/
package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
      //aligned
      m_turretSubsystem.runTurret(0);
      SmartDashboard.putBoolean("aligned", true);
      SmartDashboard.putNumber("turret speed", 0);
      return;
    } else {
      // error is Tx
      SmartDashboard.putBoolean("aligned", false);
      double kp = .05;
      double minAdjust = .1;
      double speed = Math.abs(Limelight.getTx()) * Math.signum(Limelight.getTx()) * kp;
      System.out.println(speed);
      if(Math.abs(speed) > 0.3) {
        speed = Math.signum(speed) * .3;
      }
      SmartDashboard.putNumber("turret speed", speed);
      m_turretSubsystem.runTurret(speed); 
    }
  }
}
