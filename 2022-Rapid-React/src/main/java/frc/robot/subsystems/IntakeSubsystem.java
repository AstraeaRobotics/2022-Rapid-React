// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;

public class IntakeSubsystem extends SubsystemBase {

  DoubleSolenoid left;
  DoubleSolenoid right;

  CANSparkMax mIntake;

  private final I2C.Port i2cPort;
  ColorSensorV3 colorSensor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    left = new DoubleSolenoid(15, PneumaticsModuleType.REVPH, 15, 14); // port numbers are random
    right = new DoubleSolenoid(15, PneumaticsModuleType.REVPH, 13, 12);
    left.set(DoubleSolenoid.Value.kReverse); // setting as default
    right.set(DoubleSolenoid.Value.kReverse);

    mIntake = new CANSparkMax(99, CANSparkMaxLowLevel.MotorType.kBrushless);

    i2cPort = I2C.Port.kOnboard;
    colorSensor = new ColorSensorV3(i2cPort);

  }

  public void extendAndRetract() {
    // pneumatics are doing opposite of curr status
    /*
     * if (left.get() == Value.kReverse){
     * left.set(DoubleSolenoid.Value.kForward);
     * right.set(DoubleSolenoid.Value.kForward);
     * }
     * else if (left.get() == Value.kForward) {
     * left.set(DoubleSolenoid.Value.kReverse);
     * right.set(DoubleSolenoid.Value.kReverse);
     * }
     */

    left.toggle();
    right.toggle();
  }

  public void setMotor(double speed) {
    mIntake.set(speed);
  }

  public Alliance getTeam() {
    return Alliance.Blue;
    // blue is true, red is false
    /*
     * if (colorSensor.getBlue() > colorSensor.getRed() + 30) {
     * return Alliance.Blue;
     * } else if (colorSensor.getRed() > colorSensor.getBlue() + 30) {
     * return Alliance.Red;
     * }
     * return Alliance.Invalid;
     */

  }

  public boolean isExtended() {
    return right.get() == Value.kForward;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
