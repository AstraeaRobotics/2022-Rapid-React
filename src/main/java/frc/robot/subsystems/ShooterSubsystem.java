/********************************************************************************
*                                                                               *
*   Copyright (c) Astraea Robotics, FIRST, and other WPILib contributors        *
*                                                                               *
*   Open Source Software; you can modify and/or share it under the terms of     *
*   the license file in the root directory of this project.                     *
*                                                                               *
********************************************************************************/
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.util.Logger;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ShooterSubsystem extends SubsystemBase {
  WPI_TalonFX feeder = new WPI_TalonFX(RobotMap.kFeederCAN);
  WPI_TalonFX flywheel = new WPI_TalonFX(RobotMap.kFlywheelCAN);

  PIDController feederPID = new PIDController(0, 0, 0);
  PIDController flywheelPID = new PIDController(0, 0, 0);
  
  DCMotor flywheelDCMotor = DCMotor.getFalcon500(1);
  FlywheelSim flywheelSim = new FlywheelSim(flywheelDCMotor, 1, (.5)*(Units.lbsToKilograms(2.44)*Math.pow(Units.inchesToMeters(4), 2)));
  FlywheelSim feederSim = new FlywheelSim(flywheelDCMotor, 1, (.5)*(Units.lbsToKilograms(2.44)*Math.pow(Units.inchesToMeters(4), 2)));

  double m_feederSetpoint = 0;
  double m_flywheelSetpoint = 0;

  double m_flywheelVoltageInput = 0;
  double m_feederVoltageInput = 0;

  public ShooterSubsystem() {
    feeder.setNeutralMode(NeutralMode.Coast);
    flywheel.setNeutralMode(NeutralMode.Coast);

    feeder.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, Shooter.kPIDLoopIDx, Shooter.kTimeoutMs);
    feeder.config_kF(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkF, Shooter.kTimeoutMs);
    feeder.config_kP(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkP, Shooter.kTimeoutMs);
    feeder.config_kI(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkI, Shooter.kTimeoutMs);
    feeder.config_kD(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkD, Shooter.kTimeoutMs);

    flywheel.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, Shooter.kPIDLoopIDx, Shooter.kTimeoutMs);
    flywheel.config_kF(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkF, Shooter.kTimeoutMs);
    flywheel.config_kP(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkP, Shooter.kTimeoutMs);
    flywheel.config_kI(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkI, Shooter.kTimeoutMs);
    flywheel.config_kD(Shooter.kPIDLoopIDx, Shooter.kGains_VelocitkD, Shooter.kTimeoutMs);
  }

  public void stopMotors() {
    flywheel.setVoltage(0);
    feeder.setVoltage(0);
  }

  /**
  * Changes the flywheel setpoint
  * @param setpoint the percent of max speed to change setpoint to (between 0 and 100)
  */
 public void setFlywheelSetpoint(double setpoint) {
   m_flywheelSetpoint = setpoint;
 }

  /**
   * Changes the feeder setpoint
   * @param setpoint the percent of max speed to change setpoint to (between 0 and 100)
   */
  public void setFeederSetpoint(double setpoint) {
    m_feederSetpoint = setpoint;
  }

  public double getFlywheelSetpoint() {
    return m_flywheelSetpoint;
  }

  public double getFeederSetPoint() {
    return m_feederSetpoint;
  }

  public double getFeederRPM() {
    if (RobotBase.isReal()) {
      return falconVelocityToRPM(feeder.getSelectedSensorVelocity());
    }
    return feederSim.getAngularVelocityRPM();
  }

  public double getFlywheelRPM() {
    if (RobotBase.isReal()) {
      return falconVelocityToRPM(flywheel.getSelectedSensorVelocity());
    }
    return flywheelSim.getAngularVelocityRPM();
  }

  private static double falconVelocityToRPM(double sensorVelocity) {
    return sensorVelocity * 600 / 2048;
  }

  public void setFlyWheelInputVoltage(double voltage) {
    m_flywheelVoltageInput = voltage;
  }

  public void setFeederInputVoltage(double voltage) {
    m_feederVoltageInput = voltage;
  }

  private void log() {
    Logger.log("Shooter", "Flywheel Input Voltage", m_flywheelVoltageInput);
    Logger.log("Shooter", "Flywheel Setpoint", m_flywheelSetpoint);
    Logger.log("Shooter", "Feeder Input Voltage", m_feederVoltageInput);
    Logger.log("Shooter", "Feeder Setpoint", m_flywheelSetpoint);
    if (RobotBase.isReal()) {
      Logger.log("Shooter", "Flywheel RPM", getFlywheelRPM());
      Logger.log("Shooter", "Feeder RPM", getFeederRPM());
      return;
    }
    Logger.log("Shooter", "Flywheel RPM", flywheelSim.getAngularVelocityRPM());
    Logger.log("Shooter", "Feeder RPM", feederSim.getAngularVelocityRPM());
  }

  @Override
  public void periodic() {
    flywheel.setVoltage(m_flywheelVoltageInput);
    feeder.setVoltage(m_feederVoltageInput);
    log();
  }

  @Override
  public void simulationPeriodic() {
    flywheelSim.setInputVoltage(flywheel.getMotorOutputVoltage());
    flywheelSim.update(0.02);

    feederSim.setInputVoltage(feeder.getMotorOutputVoltage());
    feederSim.update(0.02);
  }
}
