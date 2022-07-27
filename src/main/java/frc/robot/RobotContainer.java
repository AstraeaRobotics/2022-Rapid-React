// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.auto.DriveToDistance;
import frc.robot.commands.drive.SimDrive;
import frc.robot.commands.led.ToggleLED;
import frc.robot.commands.shooter.ManualShoot;
import frc.robot.commands.turret.AutoAimTurret;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  /* GAMEPADS */
  private static final PS4Controller driverGamepad = new PS4Controller(Constants.RobotMap.kDriverControllerPort);
  private static final PS4Controller operatorGamepad = new PS4Controller(Constants.RobotMap.kOperatorControllerPort);

  private static final JoystickButton triangleButton = new JoystickButton(operatorGamepad, PS4Controller.Button.kTriangle.value);

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();

  public RobotContainer() {
    configureButtonBindings();
    CommandScheduler.getInstance().schedule(new ToggleLED(m_ledSubsystem, true));
    m_shooterSubsystem.setDefaultCommand(new ManualShoot(m_shooterSubsystem, 50, 50));
    m_driveSubsystem.setDefaultCommand(
        new SimDrive(m_driveSubsystem, 2,
            driverGamepad::getR2Axis,
            driverGamepad::getL2Axis,
            driverGamepad::getLeftX,
            driverGamepad::getRightX));
  }

  private void configureButtonBindings() {
    triangleButton.whileHeld(new AutoAimTurret(m_turretSubsystem, 0.05));
  }

  public Command getAutonomousCommand() {
    return new DriveToDistance(m_driveSubsystem, 2);
  }

}
