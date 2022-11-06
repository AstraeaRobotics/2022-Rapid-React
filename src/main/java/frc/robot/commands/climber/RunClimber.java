// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.status.Status;
import frc.robot.status.Status.ClimberStatus;
import frc.robot.subsystems.ClimberSubsystem;

// for making the elevator descend
public class RunClimber extends CommandBase {
    /** Creates a new ElevatorDown. */
    private ClimberSubsystem m_climberSubsystem;
    private double m_speed;

    public RunClimber(ClimberSubsystem climberSubsystem, double speed) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_climberSubsystem = climberSubsystem;
        m_speed = speed;
        addRequirements(m_climberSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_climberSubsystem.setSpeed(m_speed);
        Status.logClimbStatus(ClimberStatus.kClimbing);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Status.logClimbStatus(ClimberStatus.kStopped);
        m_climberSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
