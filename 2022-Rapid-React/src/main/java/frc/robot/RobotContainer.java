// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SimDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Ramsete;
import frc.robot.util.Traj;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* GAMEPADS */
  public static final PS4Controller driverGamepad = new PS4Controller(Constants.RobotMap.DRIVER_CONTROLLER_PORT);
  public static final PS4Controller operatorGamepad = new PS4Controller(Constants.RobotMap.OPERATOR_CONTROLLER_PORT);

  /* Subsystems */
  //public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public static final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public static final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
    //m_driveSubsystem.setDefaultCommand(new SimDrive());
  }

  private void configureButtonBindings() {
     //Example command
     JoystickButton crossButton = new JoystickButton(operatorGamepad, 2);
     crossButton.whenPressed(new ShooterCommand(m_visionSubsystem, m_shooterSubsystem, crossButton));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //RobotContainer.m_driveSubsystem.resetOdometry(Traj.createNewTrajectoryFromJSON("OneBall-1").getInitialPose());

    // return new SequentialCommandGroup(
    //   Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("OneBall-1"), m_driveSubsystem, true)
    // );
    return null;
  }
}
