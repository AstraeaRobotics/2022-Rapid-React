// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class ToggleLED extends CommandBase {

  LEDSubsystem m_ledSubsystem;
  boolean enable;

  public ToggleLED(LEDSubsystem ledSubsystem, boolean enable) {
    addRequirements(ledSubsystem);
    this.enable = enable;
    this.m_ledSubsystem = ledSubsystem;

    // ShuffleboardTab dashboard = Shuffleboard.getTab("Dashboard");
    // dashboard.add("LEDStatus", this.m_ledSubsystem);
  }

  @Override
  public void initialize() {
    m_ledSubsystem.setIsEnabled(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
