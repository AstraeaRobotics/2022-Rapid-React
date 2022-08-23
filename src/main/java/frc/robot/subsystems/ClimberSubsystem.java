// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private CANSparkMax m_climberMotor;
  private DigitalInput m_limitSwitch;
  public double m_climbSpeed;
  private final RelativeEncoder m_encoder;
  PIDController pid;
  ElevatorSim m_elevatorSimulation;
  EncoderSim m_encoderSimulation;
  private final Mechanism2d m_mech2d;
  private final MechanismRoot2d m_mech2dRoot;
  private final MechanismLigament2d m_elevatorMech2d;

  public ClimberSubsystem() {
    m_climberMotor = new CANSparkMax(Climber.kClimberMotor_Port, MotorType.kBrushless);
    m_limitSwitch = new DigitalInput(Climber.kLimitSwitch_Port);
    pid = new PIDController(Climber.kP, Climber.kI, Climber.kD);
    m_encoder = m_climberMotor.getEncoder();
    m_encoder.setPosition(0);
    m_climbSpeed = Climber.kElevatorSpeed;
    m_elevatorSimulation = new ElevatorSim(DCMotor.getNEO(1), 100, 45, Units.inchesToMeters(2), 0, 1);
    m_encoderSimulation = new EncoderSim(new Encoder(1, 2));
    m_mech2d = new Mechanism2d(20, 50);
    m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 2);
    m_elevatorMech2d = m_mech2dRoot.append(
        new MechanismLigament2d("Elevator", Units.metersToInches(m_elevatorSimulation.getPositionMeters()), 90));
    SmartDashboard.putData("Elevator Sim", m_mech2d);
  }

  @Override
  public void periodic() {
    // setSpeed(0.2);
    SmartDashboard.putNumber("Elevator Sim Position", m_elevatorSimulation.getPositionMeters());
  }

  public boolean fullyRetracted() {
    return m_limitSwitch.get();
  }

  public void setSoftLimits() {
    m_climberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_climberMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_climberMotor.setSoftLimit(SoftLimitDirection.kForward, Climber.kUpperLimit); // 15
    m_climberMotor.setSoftLimit(SoftLimitDirection.kReverse, Climber.kLowerLimit); // 0
  }

  public void setSpeed(double speed) {
    m_climberMotor.set(speed);
  }

  public void setPIDSpeed(double point) {
    m_climberMotor.set(pid.calculate(m_encoder.getPosition(), point));
    SmartDashboard.putNumber("PID", pid.calculate(m_encoder.getPosition(), point));
  }

  public void reset() {
    m_encoder.setPosition(0);
  }

  public void climb() {
    setPIDSpeed(1);
  }

  public void descend() {
    setPIDSpeed(0);
  }

  public void stop() {
    m_climberMotor.set(0);
  }

  public void log() {
    SmartDashboard.putNumber("Current Position", Double.valueOf(String.valueOf(m_climberMotor.getEncoder())));
    SmartDashboard.putNumber("Upper Limit", Climber.kUpperLimit);
    SmartDashboard.putNumber("Lower Limit", Climber.kLowerLimit);
  }

  @Override
  public void simulationPeriodic() {
    m_elevatorSimulation.setInput(m_climberMotor.get() * RobotController.getBatteryVoltage());
    m_elevatorSimulation.update(0.020);
    m_encoderSimulation.setDistance(m_elevatorSimulation.getPositionMeters());
    m_elevatorMech2d.setLength(Units.metersToInches(m_elevatorSimulation.getPositionMeters()));

  }
}