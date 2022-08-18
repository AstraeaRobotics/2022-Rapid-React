// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class Toggle extends CommandBase {
  /** Creates a new Toggle. */
  private ClimberSubsystem m_climberSubsystem;

  public Toggle(ClimberSubsystem climberSubsystem) {
    addRequirements(m_climberSubsystem);
    m_climberSubsystem.setIsRising(true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_climberSubsystem.getButtonPressed() == false) {
      m_climberSubsystem.stop();
    } else if (m_climberSubsystem.getIsRising() && m_climberSubsystem.getButtonPressed()) {
      m_climberSubsystem.ascend();
    } else if (!m_climberSubsystem.getIsRising() && m_climberSubsystem.getButtonPressed()) {
      m_climberSubsystem.descend();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.stop();
    m_climberSubsystem.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
