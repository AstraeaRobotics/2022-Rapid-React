/********************************************************************************
*                                                                               *
*   Copyright (c) Astraea Robotics, FIRST, and other WPILib contributors        *
*                                                                               *
*   Open Source Software; you can modify and/or share it under the terms of     *
*   the license file in the root directory of this project.                     *
*                                                                               *
********************************************************************************/
package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkMax {

  /**
   * Factory method to create a SparkMax object
   *
   * @param canID the CAN ID of the SparkMax
   * @param brushless if the motor is brushless or not
   * @return a SparkMax object
   */
  public static CANSparkMax constructSparkMax(int canID, boolean brushless) {
    if (brushless) {
      return new CANSparkMax(canID, MotorType.kBrushless);
    } else {
      return new CANSparkMax(canID, MotorType.kBrushed);
    }
  }
}
