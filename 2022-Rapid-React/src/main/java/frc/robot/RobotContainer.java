// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;

public class RobotContainer {

  /* GAMEPADS */
  public static final PS4Controller driverGamepad = new PS4Controller(Constants.RobotMap.DRIVER_CONTROLLER_PORT);
  public static final PS4Controller operatorGamepad = new PS4Controller(Constants.RobotMap.OPERATOR_CONTROLLER_PORT);

  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

}
