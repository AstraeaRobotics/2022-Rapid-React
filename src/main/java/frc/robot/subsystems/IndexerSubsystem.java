// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.status.Status;
import frc.robot.status.Status.IntakeStatus;

public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {}

  CANSparkMax belt = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkMax transition = new CANSparkMax(8, MotorType.kBrushless);


  public void spinMotors(double speed) {
    belt.set(-speed);
    transition.set(speed);
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
