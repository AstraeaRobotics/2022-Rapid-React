/********************************************************************************
*                                                                               *
*   Copyright (c) Astraea Robotics, FIRST, and other WPILib contributors        *
*                                                                               *
*   Open Source Software; you can modify and/or share it under the terms of     *
*   the license file in the root directory of this project.                     *
*                                                                               *
********************************************************************************/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase implements Sendable {

  DoubleSolenoid left;
  DoubleSolenoid right;

  CANSparkMax m_motor;

  NetworkTableEntry m_intakeEntry;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    left = new DoubleSolenoid(15, PneumaticsModuleType.REVPH, 15, 14); // port numbers are random
    right = new DoubleSolenoid(15, PneumaticsModuleType.REVPH, 12, 13);
    left.set(DoubleSolenoid.Value.kReverse); // setting as default
    right.set(DoubleSolenoid.Value.kReverse);

    m_motor = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);

    ShuffleboardTab driveTab = Shuffleboard.getTab("Dashboard");
    m_intakeEntry = driveTab.add("Intake Status", isExtended()).withSize(3, 1).withPosition(10, 2).getEntry();
    // driveTab.getLayout("Intake Status", BuiltInLayouts.kGrid).withSize(3, 1).withPosition(10, 2).withProperties(Map.of("Label position", "HIDDEN"));

    ShuffleboardTab testTab = Shuffleboard.getTab("Testing");
    testTab.add("Intake Status", this);
    testTab.getLayout("Intake Status", BuiltInLayouts.kGrid).withSize(1, 2).withPosition(3, 0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake State");
    builder.addBooleanProperty("Left", this::isExtendedLeft, null);
    builder.addBooleanProperty("Right", this::isExtended, null);
  }


  public void toggleIntake() {
    left.toggle();
    right.toggle();

    m_intakeEntry.setBoolean(isExtended());
  }

  public void setMotor(double speed) {
    m_motor.set(speed);
  }

  public boolean isExtended() {
    return right.get() == Value.kForward;
  }

  public boolean isExtendedLeft() {
    return left.get() == Value.kForward;
  }
}
