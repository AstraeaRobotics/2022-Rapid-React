/********************************************************************************
 * * Copyright (c) Astraea Robotics, FIRST, and other WPILib contributors * * Open Source Software;
 * you can modify and/or share it under the terms of * the license file in the root directory of
 * this project. * *
 ********************************************************************************/
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  DoubleSolenoid left;
  DoubleSolenoid right;
  CANSparkMax m_motor;

  public IntakeSubsystem() {
    left = new DoubleSolenoid(15, PneumaticsModuleType.REVPH, 15, 14); // port numbers are random
    right = new DoubleSolenoid(15, PneumaticsModuleType.REVPH, 12, 13);
    left.set(DoubleSolenoid.Value.kReverse); // setting as default
    right.set(DoubleSolenoid.Value.kReverse);
    m_motor = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
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

  public void periodic() {
    if(!isExtended()) {
      setMotor(0);
    }
  }
}
