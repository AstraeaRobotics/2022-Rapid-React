// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.NavigableMap;
import java.util.TreeMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.status.Status;
import frc.robot.status.Status.IntakeStatus;
import frc.robot.util.SparkMax;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;


public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new IndexerSubsystem. */
  public ColorSensorV3 lowerSensor;
  public ColorSensorV3 upperSensor;
  public static I2C.Port I2C;

  CANSparkMax belt = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkMax transition = new CANSparkMax(8, MotorType.kBrushless);

  public IndexerSubsystem() {
    lowerSensor = new ColorSensorV3(Port.kOnboard);
    upperSensor = new ColorSensorV3(Port.kOnboard);
  }

  public void spinBelt(double speed) {
    belt.set(-speed);
  }

  public void spinTransition(double speed) {
    transition.set(speed);
  }

  /*
   * @param sensor upper or lower sensor
   * @param ballNumber 0 for lower ball, 1 for upper ball
  */
  public void getDetectedColor(ColorSensorV3 sensor, int ballNumber) {
    int red = sensor.getRed();
    int blue = sensor.getBlue();
    if (getProximity(sensor) < Constants.Indexer.sensorDist) { //If the ball is too far away, or under the distance value, return null
      logBallStatus(ballNumber, Static.BallStatus.kEmpty);
    } else if (red > blue) {
      logBallStatus(ballNumber, Static.BallStatus.kRed);
    } else if (red < blue) {
      logBallStatus(ballNumber, Static.BallStatus.kBlue);
    } else {
      logBallStatus(ballNumber, Static.BallStatus.kEmpty);
    }
  }

  public int getProximity(ColorSensorV3 sensor) {
    return sensor.getProximity();
  }

  @Override
  public void periodic() {
    if(Status.getIntakeStatus() == IntakeStatus.kExtended) {
      transition.set(0.5);
    } else {
      transition.set(0.0);
    }
  }
}
