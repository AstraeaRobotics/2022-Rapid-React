// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SimDrive extends CommandBase {

  private DriveSubsystem m_subsystem;
  private SlewRateLimiter m_slewRateLimiter;

  private DoubleSupplier m_forwardsSupplier;
  private DoubleSupplier m_backwardsSupplier;
  private DoubleSupplier m_curveSupplier;
  private DoubleSupplier m_turnSupplier;

  /**
   * Construct a new Sim Drive command
   * 
   * @param subsystem               Subsystem to require
   * @param rateLimit               Maximum rate of change of controller inputs
   * @param forwardsSupplier        Supplier to get forwards input value, likely from a trigger axis
   * @param backwardsSupplier       Supplier to get backwards input value, likely from a trigger axis
   * @param curveAxisSupplier       Supplier to get curve drive input value, likely from a joystick axis
   * @param turnInPlaceAxisSupplier Supplier to get turn in place input value, likely from a joystick axis
   */
  public SimDrive(DriveSubsystem subsystem, double rateLimit, DoubleSupplier forwardsSupplier,
      DoubleSupplier backwardsSupplier, DoubleSupplier curveAxisSupplier,
      DoubleSupplier turnInPlaceAxisSupplier) {
    m_subsystem = subsystem;
    m_slewRateLimiter = new SlewRateLimiter(rateLimit);
    m_forwardsSupplier = forwardsSupplier;
    m_backwardsSupplier = backwardsSupplier;
    m_curveSupplier = curveAxisSupplier;
    m_turnSupplier = turnInPlaceAxisSupplier;

    addRequirements(m_subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double valetSpeed = 1;

    double speed = (m_forwardsSupplier.getAsDouble() - m_backwardsSupplier.getAsDouble()) * valetSpeed * (-1);
    double adjustedSpeed = m_slewRateLimiter.calculate(speed);

    
    m_subsystem.curveDrive(adjustedSpeed, m_curveSupplier.getAsDouble(), false);

    double slowTurnSpeed = m_turnSupplier.getAsDouble();
    if (Math.abs(slowTurnSpeed) > DriveConstants.kDeadzone) {
      m_subsystem.tankDriveRaw(slowTurnSpeed * DriveConstants.kTurnSpeed,
          -slowTurnSpeed * DriveConstants.kTurnSpeed);
    }
  }
}
