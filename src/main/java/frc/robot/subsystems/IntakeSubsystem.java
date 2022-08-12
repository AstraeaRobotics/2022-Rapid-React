// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class IntakeSubsystem extends SubsystemBase {

  DoubleSolenoid left;
  DoubleSolenoid right;

  CANSparkMax m_motor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    left = new DoubleSolenoid(15, PneumaticsModuleType.REVPH, 15, 14); // port numbers are random
    right = new DoubleSolenoid(15, PneumaticsModuleType.REVPH, 12, 13);
    left.set(DoubleSolenoid.Value.kReverse); // setting as default
    right.set(DoubleSolenoid.Value.kReverse);

    m_motor = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);

    ShuffleboardTab driveTab = Shuffleboard.getTab("Dashboard");
    driveTab.add("Intake Status", left);
    // driveTab.add("Intake Status", isExtended()).withWidget(BuiltInWidgets.kBooleanBox);

    ShuffleboardTab testTab = Shuffleboard.getTab("Testing");
    testTab.add("Left Intake Status", left);
    testTab.add("Right Intake Status", right);
  }

  public void toggleIntake() {
    left.toggle();
    right.toggle();
  }

  public void setMotor(double speed) {
    m_motor.set(speed);
  }

  public boolean isExtended() {
    return right.get() == Value.kForward;
  }
}