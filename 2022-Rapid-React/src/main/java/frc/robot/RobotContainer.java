// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SimDrive;
import frc.robot.commands.ManualTurret;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Ramsete;
import frc.robot.util.Traj;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.AutoAimTurret;

public class RobotContainer {

  /* GAMEPADS */
  public static final PS4Controller driverGamepad = new PS4Controller(Constants.RobotMap.DRIVER_CONTROLLER_PORT);
  public static final PS4Controller operatorGamepad = new PS4Controller(Constants.RobotMap.OPERATOR_CONTROLLER_PORT);

  /* BUTTONS */
  private final JoystickButton circleButton = new JoystickButton(driverGamepad, 3);
  private final JoystickButton squareButton = new JoystickButton(driverGamepad, 1);
  private final JoystickButton triangleButton = new JoystickButton(driverGamepad, 4);

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();

  public RobotContainer() {
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(new SimDrive());
  }

  private void configureButtonBindings() {
    squareButton.whileHeld(new ManualTurret(m_turretSubsystem, -0.05));
    circleButton.whileHeld(new ManualTurret(m_turretSubsystem, 0.05));
    triangleButton.whileHeld(new AutoAimTurret(m_turretSubsystem));
  }

  public Command getAutonomousCommand() {
    RobotContainer.m_driveSubsystem.resetOdometry(Traj.createNewTrajectoryFromJSON("OneBall-1").getInitialPose());

    return new SequentialCommandGroup(
        Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("OneBall-1"), m_driveSubsystem, true));
  }
}
