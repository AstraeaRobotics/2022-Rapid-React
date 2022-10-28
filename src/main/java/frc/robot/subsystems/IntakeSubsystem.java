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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ColorSensorV3;

public class IntakeSubsystem extends SubsystemBase {

  DoubleSolenoid left;
  DoubleSolenoid right;

  ColorSensorV3 colorSensor;
  private final I2C.Port i2cPort;

  Alliance team;

  CANSparkMax m_motor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    left = new DoubleSolenoid(15, PneumaticsModuleType.REVPH, 15, 14); // port numbers are random
    right = new DoubleSolenoid(15, PneumaticsModuleType.REVPH, 12, 13);
    left.set(DoubleSolenoid.Value.kReverse); // setting as default
    right.set(DoubleSolenoid.Value.kReverse);

    m_motor = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);

    i2cPort = I2C.Port.kOnboard;
    colorSensor = new ColorSensorV3(i2cPort);
    team = DriverStation.getAlliance();

  }

  public void toggleIntake() {
    left.toggle();
    right.toggle();
  }

  public void setMotor(double speed) {
    if (isExtended()) {
      m_motor.set(speed);
    } else {
      m_motor.set(0);
    }
  }

  public Alliance getBallColor() {
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

  public boolean isExtended() {
    return right.get() == Value.kForward;
  }
}
