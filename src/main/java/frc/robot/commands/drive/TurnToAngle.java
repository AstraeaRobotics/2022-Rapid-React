// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends PIDCommand {

  private final double setpoint;
  private final DriveSubsystem m_driveSubsystem;

  public TurnToAngle(double targetAngleDegrees, DriveSubsystem drive) {
    super(
      new PIDController(.05, 0, 0),
      drive::getHeading,
      targetAngleDegrees + drive.getHeading(),
      output -> drive.setOutput(MathUtil.clamp(output, -10, 10), -MathUtil.clamp(output, -10, 10)),
      drive
      );
    this.setpoint = targetAngleDegrees;
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(1, 2);
    this.m_driveSubsystem = drive;
  }

  @Override
  public void initialize() {
    double heading = m_driveSubsystem.getHeading();
    super.m_setpoint = () -> ( heading + setpoint);
    System.out.println(super.m_setpoint.getAsDouble());
  }

  @Override
  public void execute() {
      // TODO Auto-generated method stub
      super.execute();

      System.out.println("Setpoint: " + super.m_setpoint.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}