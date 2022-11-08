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

public class IndexerSubsystem extends SubsystemBase {

  CANSparkMax belt = new CANSparkMax(Constants.RobotMap.kIndexerBelt, MotorType.kBrushless);
  CANSparkMax transition = new CANSparkMax(Constants.RobotMap.kIndexerTransition, MotorType.kBrushless);

  public IndexerSubsystem() {
    belt.setInverted(true);
  }

  public void spinBelt(double speed) {
    belt.set(speed);
  }

  public void spinTransition(double speed) {
    transition.set(speed);
  }

  @Override
  public void periodic() {
    if (Status.getIntakeStatus() == IntakeStatus.kExtended) {
      transition.set(Constants.Indexer.kTransitionSpeed);
    } else {
      transition.set(0.0);
    }
  }
}
