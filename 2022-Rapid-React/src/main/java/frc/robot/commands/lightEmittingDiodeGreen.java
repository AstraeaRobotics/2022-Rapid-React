// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightEmittingDiode;

public class lightEmittingDiodeGreen extends CommandBase {

  LightEmittingDiode m_LED;
  /** Creates a new lightEmittingDiodeGreen. */
  public lightEmittingDiodeGreen(LightEmittingDiode m_LED) {
    this.m_LED = m_LED;
    addRequirements(m_LED);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_LED.glowGreen();
    // RobotContainer.M_DIODE.glowRed();
    // RobotContainer.M_DIODE.glowBlue();

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
