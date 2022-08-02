// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.status.Status;
import frc.robot.status.Status.IntakeStatus;
import frc.robot.util.SparkMax;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;

public class IndexerSubsystem extends SubsystemBase {
  Alliance team;

  private final I2C.Port i2cPort;
  ColorSensorV3 colorSensor;
  CANSparkMax belt = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkMax transition = new CANSparkMax(8, MotorType.kBrushless);
  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
    i2cPort = I2C.Port.kOnboard;
    colorSensor = new ColorSensorV3(i2cPort);
    team = DriverStation.getAlliance();
  }


  public void spinMotors(double speed) {
    belt.set(-speed);
    transition.set(speed);
  }

  public Alliance getBallColor(){
    if (colorSensor.getProximity() < 150) {
      return Alliance.Invalid;
    }
    int blueColorValue = colorSensor.getBlue();
    int redColorValue = colorSensor.getRed();
    if (blueColorValue > redColorValue) {
      return Alliance.Blue;
    } else {
      return Alliance.Red;
    }
    }


  @Override
  public void periodic() {
    SmartDashboard.putString("Ball Color", getBallColor().toString());
    if(Status.getIntakeStatus() == IntakeStatus.kExtended) {
      if(DriverStation.getAlliance() != getBallColor()) {
        spinMotors(-0.5);
      } else {
        transition.set(0.5);
      }
    } else {
      transition.set(0.0);
    }

  }
}
