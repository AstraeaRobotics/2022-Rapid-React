/********************************************************************************
*                                                                               *
*   Copyright (c) Astraea Robotics, FIRST, and other WPILib contributors        *
*                                                                               *
*   Open Source Software; you can modify and/or share it under the terms of     *
*   the license file in the root directory of this project.                     *
*                                                                               *
********************************************************************************/
package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends PIDCommand {

  public TurnToAngle(double targetAngleDegrees, DriveSubsystem drive) {
    super(
        new PIDController(.05, 0, 0),
        drive::getHeading,
        drive.getHeading() + targetAngleDegrees,
        output ->
            drive.setOutput(MathUtil.clamp(output, -10, 10), -MathUtil.clamp(output, -10, 10)),
        drive);
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(1, 2);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
