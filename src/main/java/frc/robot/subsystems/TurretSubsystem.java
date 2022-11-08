/********************************************************************************
 * * Copyright (c) Astraea Robotics, FIRST, and other WPILib contributors * * Open Source Software;
 * you can modify and/or share it under the terms of * the license file in the root directory of
 * this project. * *
 ********************************************************************************/
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

  private CANSparkMax m_turretMotor;

  private RelativeEncoder m_encoder;

  private boolean locked = false;

  public TurretSubsystem() {
    m_turretMotor =
        new CANSparkMax(RobotMap.kTurretCANId, MotorType.kBrushless);
    m_encoder = m_turretMotor.getEncoder();
    m_encoder.setPosition(0.0);
    m_turretMotor.setSoftLimit(SoftLimitDirection.kReverse,
        -TurretConstants.kMaxRotation);
    m_turretMotor.setSoftLimit(SoftLimitDirection.kForward,
        TurretConstants.kMaxRotation);
    m_turretMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  public void runTurret(double speed) {
    if (!locked) {
      MathUtil.clamp(speed, -TurretConstants.kMaxSpeed,
          TurretConstants.kMaxSpeed);
      m_turretMotor.set(speed);
    }
  }

  public void setLocked(boolean isLocked) {
    this.locked = isLocked;
  }

  public boolean isLocked() {
    return locked;
  }

  public double getCurrentPosition() {
    return m_encoder.getPosition();
  }

  public void setEncoderPosition(double position) {
    m_encoder.setPosition(position);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Dist", m_encoder.getPosition());
    SmartDashboard.putBoolean("Turet Locked", isLocked());

    if(DriverStation.isDisabled()) {
      m_turretMotor.setIdleMode(IdleMode.kCoast);
    } else {
      m_turretMotor.setIdleMode(IdleMode.kBrake);
    }
  }
}
