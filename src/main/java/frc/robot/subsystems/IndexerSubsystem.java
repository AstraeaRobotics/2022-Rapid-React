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


public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new IndexerSubsystem. */
  CANSparkMax belt = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkMax transition = new CANSparkMax(8, MotorType.kBrushless);

  public IndexerSubsystem() {}

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
  public void getDetectedColor(int sensorID, int ballNumber) {
    NetworkTableEntry colorEntry = NetworkTableInstance.getDefault().getEntry("/rawcolor" + sensorID);

    double red = colorEntry.getDoubleArray(new double[]{0, 0, 0, 0})[0];
    double blue = colorEntry.getDoubleArray(new double[]{0, 0, 0, 0})[2];

    if (getProximity(sensorID) < Constants.Indexer.sensorDist) {
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
    return NetworkTableInstance.getDefault().getEntry("/prox" + sensorID).getDouble(0);
  }

  @Override
  public void periodic() {
    getDetectedColor(0, 1);
    getDetectedColor(1, 0);
    if(Status.getIntakeStatus() == IntakeStatus.kExtended) {
      transition.set(0.5);
    } else {
      transition.set(0.0);
    }
  }
}
