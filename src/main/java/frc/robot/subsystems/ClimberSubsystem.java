// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private CANSPARKMAX climber_motor = new CANSparkMax(50, MotorType.kBrushless); //Magik Number Port
  private DigitalInput limit_switch = new DigitalInput();

  public ClimberSubsystem() {
    
  }
  
  private void setElevator(double speed) {
    climber_motor(speed);
  }

  private boolean getLimitSwitch() {
    return limit_switch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}