// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode; //Motor speed/control
import com.ctre.phoenix.motorcontrol.can.TalonSRX; //Defines motor
import com.ctre.phoenix.motorcontrol.NeutralMode; //Motor brake/run

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShootingSubsystem. */
  TalonSRX lowSrx = new TalonSRX(10);
  TalonSRX topSrx = new TalonSRX(11);
  
  public ShooterSubsystem() {
    lowSrx.setNeutralMode(NeutralMode.Coast); //Sets motor to On/Coast
    topSrx.setNeutralMode(NeutralMode.Coast);
  }

  public void setMotors(double top, double bottom) {
    lowSrx.set(ControlMode.PercentOutput, top);
    topSrx.set(ControlMode.PercentOutput, bottom);
  }

  public void stopMotors() {
    lowSrx.set(ControlMode.PercentOutput, 0.0);
    topSrx.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
