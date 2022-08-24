// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RunIndexer;
import frc.robot.commands.auto.DriveToDistance;
import frc.robot.commands.drive.SimDrive;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.led.ToggleLED;
import frc.robot.commands.shooter.ManualShoot;
import frc.robot.commands.turret.AutoAimTurret;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;


public class RobotContainer {

  /* GAMEPADS */
  private static final PS4Controller driverGamepad = new PS4Controller(
    Constants.RobotMap.kDriverControllerPort
  );
  private static final PS4Controller operatorGamepad = new PS4Controller(
    Constants.RobotMap.kOperatorControllerPort
  );

  private static final JoystickButton triangleButton = new JoystickButton(
    operatorGamepad,
    PS4Controller.Button.kTriangle.value
  );

  private static final JoystickButton X_BUTTON = new JoystickButton(driverGamepad, PS4Controller.Button.kCross.value);

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();

  private final JoystickButton m_circleButton = new JoystickButton(
    driverGamepad,
    PS4Controller.Button.kCircle.value
  );

  public RobotContainer() {
    configureButtonBindings();
    startCamStream();
    CommandScheduler
      .getInstance()
      .schedule(new ToggleLED(m_ledSubsystem, true));
    m_shooterSubsystem.setDefaultCommand(new ManualShoot(m_shooterSubsystem, 40, 40));
    m_driveSubsystem.setDefaultCommand(
      new SimDrive(
        m_driveSubsystem,
        2,
        driverGamepad::getR2Axis,
        driverGamepad::getL2Axis,
        driverGamepad::getLeftX,
        driverGamepad::getRightX
      )
    );
    m_intakeSubsystem.setDefaultCommand(new IntakeRun(m_intakeSubsystem));

  }

  private void configureButtonBindings() {
    triangleButton.whileHeld(new AutoAimTurret(m_turretSubsystem, 0.05));
    m_circleButton.whenPressed(new ToggleIntake(m_intakeSubsystem));
    X_BUTTON.whileHeld(new RunIndexer(m_indexerSubsystem));
  }

  public Command getAutonomousCommand() {
    return new DriveToDistance(m_driveSubsystem, 2);
  }

  public void startCamStream() {
    ShuffleboardTab dash = Shuffleboard.getTab("Dashboard");

    CameraServer.startAutomaticCapture();
    dash.add("Camera", CameraServer.getServer().getSource()).withWidget(BuiltInWidgets.kCameraStream)
                .withSize(5, 3).withPosition(5, 3).withProperties(Map.of("Show Crosshair", false, "Show Controls", false));
    HttpCamera limelight = new HttpCamera("limelight", "http://limelight.local:5801/stream.mjpg");
    dash.add("Limelight", limelight).withWidget(BuiltInWidgets.kCameraStream)
                .withSize(5, 3).withPosition(5, 0).withProperties(Map.of("Show Crosshair", true, "Show Controls", false));
  }
}
