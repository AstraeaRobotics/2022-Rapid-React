// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class IntakeToggle extends CommandBase {
  IntakeSubsystem m_IntakeSubsystem;
  boolean isBlueTeam;
  
  /** Creates a new IntakeToggle. */
  public IntakeToggle(IntakeSubsystem subsystem) {
    m_IntakeSubsystem = subsystem;
    if (DriverStation.getAlliance() == Alliance.Blue){
      isBlueTeam = true;
    }
    else if (DriverStation.getAlliance() == Alliance.Red){
      isBlueTeam = false;
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.extendAndRetract();  // extending or retracting pneumatics
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // d(^v^)b
    // (#-_-)
    //
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
