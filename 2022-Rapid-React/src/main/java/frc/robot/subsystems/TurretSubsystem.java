package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.RobotMap;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;

public class TurretSubsystem extends SubsystemBase {

    private CANSparkMax m_turretMotor;

    private RelativeEncoder m_encoder;

    public TurretSubsystem() {
        m_turretMotor = new CANSparkMax(RobotMap.kTurretCANId, MotorType.kBrushless);
        m_encoder = m_turretMotor.getEncoder();
        m_encoder.setPosition(0.0);
    }

    public void runTurret(double speed) {
        MathUtil.clamp(speed, -TurretConstants.kMaxSpeed, TurretConstants.kMaxSpeed);
        m_turretMotor.set(speed);
    }

    public double getCurrentPosition() {
        return m_encoder.getPosition();
    }

    public void setEncoderPosition(double position) {
        m_encoder.setPosition(position);
    }

    @Override
    public void periodic() {
    }
}