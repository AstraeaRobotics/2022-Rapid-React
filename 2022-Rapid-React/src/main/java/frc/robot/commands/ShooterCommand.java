// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import pabeles.concurrency.ConcurrencyOps.NewInstance;
import edu.wpi.first.math.controller.PIDController;


public class ShooterCommand extends CommandBase {
  /** Creates a new ShooterCommand. */
  private VisionSubsystem visionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private JoystickButton cross;
  private PIDController pidController;
  public ShooterCommand(VisionSubsystem mVisionsubsystem, ShooterSubsystem sSubsystem, JoystickButton cross) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mVisionsubsystem);
    addRequirements(sSubsystem);
    this.visionSubsystem = mVisionsubsystem;
    this.shooterSubsystem = sSubsystem;
    this.cross = cross;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PIDController pidController = new PIDController(0.1,0.0,0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pidController.setSetpoint(0.5);
    double speedTop = shooterSubsystem.getSpeed()[1];
    double speedBottom = shooterSubsystem.getSpeed()[0];
    double newSpeedTop = pidController.calculate(speedTop);
    double newSpeedBottom = pidController.calculate(speedBottom);

    shooterSubsystem.setMotors(newSpeedTop,newSpeedBottom);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopMotors();
    pidController.close();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (cross.get() == false) {
      return true;
    }
    return false;
  }
}
