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
  public static ColorSensorV3 sensor;
  public static I2C.Port I2C;

  private Color upperBall;
  private Color lowerBall;

  public IndexerSubsystem() {
    sensor = new ColorSensorV3(Port.kOnboard);
    upperBall = null;
    lowerBall = null;
  }

  CANSparkMax belt = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkMax transition = new CANSparkMax(8, MotorType.kBrushless);


  public void spinBelt(double speed) {
    belt.set(-speed);
    if (speed > 0) {
      launched();
    }
  }

  public void spinTransition(double speed) {
    transition.set(speed);
  }

  public Color getDetectedColor() {
    int red = sensor.getRed();
    int blue = sensor.getBlue();
    if (red > blue)
      return Color.kRed;
    if (red < blue)
      return Color.kBlue;
    return null;
  }

  public int getProximity() {
    return sensor.getProximity();
  }

  //0 means upper, and 1 means lower
  public void addColor(int position, Color color) {
    if (position == 0) {
      upperBall = color;
    } else if (position == 1) {
      lowerBall = color;
    }
  }

  public Color getUpperColors() {
    return upperBall;
  }

  public Color getLowerColors() {
    return lowerBall;
  }

  public void launched() { //Assume both balls are launched for now
    upperBall = null;
    lowerBall = null;
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
