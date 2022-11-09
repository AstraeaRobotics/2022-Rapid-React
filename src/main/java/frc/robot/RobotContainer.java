// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.indexer.ShootIndexer;
import frc.robot.commands.ReverseAll;
import frc.robot.commands.auto.TwoBall;
import frc.robot.commands.climber.RunClimber;
import frc.robot.commands.drive.AutoAim;
import frc.robot.commands.drive.SimDrive;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.shooter.ManualShoot;
import frc.robot.commands.turret.AutoAimTurret;
import frc.robot.commands.turret.CenterTurret;
import frc.robot.commands.turret.ManualTurret;
import frc.robot.commands.turret.RecalibrateTurret;
import frc.robot.commands.turret.ToggleTurretLock;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {

        /* GAMEPADS */
        private static final PS4Controller driverGamepad = new PS4Controller(Constants.RobotMap.kDriverControllerPort);
        private static final PS4Controller operatorGamepad = new PS4Controller(Constants.RobotMap.kOperatorControllerPort);

        /* DRIVER BUTTONS */
        private static final JoystickButton m_squareButton = new JoystickButton(driverGamepad, PS4Controller.Button.kSquare.value);
        private static final JoystickButton m_l1_button = new JoystickButton(driverGamepad, PS4Controller.Button.kL1.value);
        private static final JoystickButton m_r1_button = new JoystickButton(driverGamepad, PS4Controller.Button.kR1.value);
        private final JoystickButton m_driver_PS_Button = new JoystickButton(driverGamepad, PS4Controller.Button.kPS.value);
        private final JoystickButton m_driver_Touchpad = new JoystickButton(driverGamepad, PS4Controller.Button.kTouchpad.value);

        /* OPERATOR BUTTONS */
        private static final JoystickButton m_operator_r2_button = new JoystickButton(operatorGamepad, PS4Controller.Button.kR2.value);
        private static final JoystickButton m_operator_l2_button = new JoystickButton(operatorGamepad, PS4Controller.Button.kL2.value);
        private static final JoystickButton m_operator_circle_button = new JoystickButton(operatorGamepad, PS4Controller.Button.kCircle.value);
        private static final JoystickButton m_operator_triangle_button = new JoystickButton(operatorGamepad, PS4Controller.Button.kTriangle.value);
        private static final JoystickButton m_operator_squareButton = new JoystickButton(operatorGamepad, PS4Controller.Button.kSquare.value);
        private static final JoystickButton m_operator_cross_Button = new JoystickButton(operatorGamepad, PS4Controller.Button.kCross.value);

        private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
        private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
        private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
        private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
        private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
        private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
        private final LEDSubsystem m_LED = new LEDSubsystem();

        public RobotContainer() {
                configureButtonBindings();
                m_shooterSubsystem.setDefaultCommand(new ManualShoot(m_shooterSubsystem, 30, 40));
                m_driveSubsystem.setDefaultCommand(new SimDrive(m_driveSubsystem, 2, driverGamepad::getR2Axis, driverGamepad::getL2Axis, driverGamepad::getLeftX, driverGamepad::getRightX));
                m_intakeSubsystem.setDefaultCommand(new IntakeRun(m_intakeSubsystem));
                m_turretSubsystem.setDefaultCommand(new AutoAimTurret(m_turretSubsystem, Constants.TurretConstants.kMaxSpeed));
        }

        private void configureButtonBindings() {
                /* DRIVER CONTROLS */
                m_squareButton.whileHeld(new ShootIndexer(m_indexerSubsystem));
                m_l1_button.whileHeld(new ManualTurret(m_turretSubsystem, -Constants.TurretConstants.kMaxManualSpeed));
                m_r1_button.whileHeld(new ManualTurret(m_turretSubsystem, Constants.TurretConstants.kMaxManualSpeed));
                m_driver_PS_Button.whenPressed(new ToggleTurretLock(m_turretSubsystem));
                m_driver_Touchpad.whileHeld(new AutoAim(m_driveSubsystem));

                /* OPERATOR CONTROLS */
                m_operator_l2_button.whileHeld(new RunClimber(m_climberSubsystem, -Constants.Climber.kElevatorSpeed));
                m_operator_r2_button.whileHeld(new RunClimber(m_climberSubsystem, Constants.Climber.kElevatorSpeed));
                m_operator_squareButton.whenPressed(new ToggleIntake(m_intakeSubsystem));
                m_operator_circle_button.whileHeld(new ReverseAll(m_intakeSubsystem, m_indexerSubsystem));
                m_operator_triangle_button.whileHeld(new RecalibrateTurret(m_turretSubsystem));
                m_operator_cross_Button.whileHeld(new CenterTurret(m_turretSubsystem));
        }

        public Command getAutonomousCommand() {
                // return new OneBall(m_turretSubsystem, m_driveSubsystem, m_indexerSubsystem);
                // return new TurnToAngle(180, m_driveSubsystem);
                return new TwoBall(m_driveSubsystem, m_turretSubsystem, m_indexerSubsystem, m_intakeSubsystem);
        }

}
