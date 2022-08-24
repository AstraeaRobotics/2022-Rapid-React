// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class ToggleLED extends CommandBase {

  LEDSubsystem m_ledSubsystem;

  public ToggleLED(LEDSubsystem ledSubsystem) {
    addRequirements(ledSubsystem);
    this.m_ledSubsystem = ledSubsystem;
    System.out.println("IM HERE TOO");
  }

  @Override
  public void initialize() {
    m_ledSubsystem.toggleLED();
    System.out.println("HI I AM SOM");
  }

  @Override
  public void execute() {
    System.out.println("hello");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
