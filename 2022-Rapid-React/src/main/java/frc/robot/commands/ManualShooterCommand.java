// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Limelight;
import pabeles.concurrency.ConcurrencyOps.NewInstance;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class ManualShooterCommand extends CommandBase {
  /** Creates a new ShooterCommand. */
  private Limelight limelight;
  private ShooterSubsystem shooterSubsystem;

  public ManualShooterCommand(Limelight limelight, ShooterSubsystem sSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sSubsystem);
    this.limelight = limelight;
    this.shooterSubsystem = sSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Top and bottom shooter speeds
    
    double speed = 100;
    
    shooterSubsystem.setSpeedLower(speed);
    shooterSubsystem.setSpeedUpper(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("ShooterCommand", "Ending");
    shooterSubsystem.setMotors(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
