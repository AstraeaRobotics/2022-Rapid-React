// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class IntakeRun extends CommandBase {
  /** Creates a new IntakeRun. */
  IntakeSubsystem m_IntakeSubsystem;
  Alliance team;

  public IntakeRun(IntakeSubsystem system) {
    m_IntakeSubsystem = system;
    team = DriverStation.getAlliance();
    // System.out.println(team);
    addRequirements(m_IntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if (m_IntakeSubsystem.getTeam() == team && m_IntakeSubsystem.isExtended()
    // && m_IntakeSubsystem.getTeam() != Alliance.Invalid) {
    // m_IntakeSubsystem.setMotor(0.5);
    // } else if (m_IntakeSubsystem.getTeam() != team &&
    // m_IntakeSubsystem.isExtended()) {
    // m_IntakeSubsystem.setMotor(-0.5);
    // } else {
    // m_IntakeSubsystem.setMotor(0);
    // }

    if (m_IntakeSubsystem.isExtended()) {
      m_IntakeSubsystem.setMotor(0.5);
      System.out.println("is extended");
    } else {
      m_IntakeSubsystem.setMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
