// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.status.Status;
import frc.robot.status.Status.IntakeStatus;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import com.revrobotics.ColorSensorV3;


public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new IndexerSubsystem. */
  CANSparkMax belt = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkMax transition = new CANSparkMax(8, MotorType.kBrushless);

  private ColorSensorV3 onboardSensor;

  public IndexerSubsystem() {
    onboardSensor = new ColorSensorV3(Port.kOnboard);
    belt.setInverted(true);
  }

  public void spinBelt(double speed) {
    belt.set(speed);
  }

  public void spinTransition(double speed) {
    transition.set(speed);
  }

  /*
   * @param sensor upper or lower sensor
   * @param ballNumber 0 for lower ball, 1 for upper ball
  */
  private void getDetectedColor(int sensorID, int ballNumber) {
    double[] colorEntry = NetworkTableInstance.getDefault().getTable("").getEntry("rawcolor" + sensorID).getDoubleArray(new double[]{0, 0, 0, 0});

    double red = colorEntry[0]; //Prevent errors with calling entries that don't exist, which there are 4
    double blue = colorEntry[2];

    if (getProximity(sensorID) < Constants.Indexer.proximityThreshold) {
      Status.logBallStatus(ballNumber, Status.BallStatus.kEmpty);
    } else if (red > blue) {
      Status.logBallStatus(ballNumber, Status.BallStatus.kRed);
    } else if (red < blue) {
      Status.logBallStatus(ballNumber, Status.BallStatus.kBlue);
    } else {
      Status.logBallStatus(ballNumber, Status.BallStatus.kEmpty);
    }
  }

  private void getDetectedColorRio(int ballNumber) {
    int red = onboardSensor.getRed();
    int blue = onboardSensor.getBlue();
    int proximity = onboardSensor.getProximity();

    if (proximity < Constants.Indexer.proximityThreshold) {
      Status.logBallStatus(ballNumber, Status.BallStatus.kEmpty);
    } else if (red > blue) {
      Status.logBallStatus(ballNumber, Status.BallStatus.kRed);
    } else if (red < blue) {
      Status.logBallStatus(ballNumber, Status.BallStatus.kBlue);
    } else {
      Status.logBallStatus(ballNumber, Status.BallStatus.kEmpty);
    }
  }

  public double getProximity(int sensorID) {
    return NetworkTableInstance.getDefault().getTable("").getEntry("proximity" + sensorID).getDouble(0);
  }

  @Override
  public void periodic() {
    getDetectedColorRio(0);
    getDetectedColor(1, 1);
    if(Status.getIntakeStatus() == IntakeStatus.kExtended) {
      transition.set(Constants.Indexer.transitionSpeed);
    } else {
      transition.set(0.0);
    }
  }
}
