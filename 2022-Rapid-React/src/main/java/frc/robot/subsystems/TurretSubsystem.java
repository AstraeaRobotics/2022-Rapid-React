package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.Logger;

public class TurretSubsystem extends SubsystemBase {

    private CANSparkMax m_turretMotor;

    private RelativeEncoder m_encoder;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("Turret");


    public TurretSubsystem() {
        m_turretMotor = new CANSparkMax(TurretConstants.TurretCANid, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_encoder = m_turretMotor.getEncoder();
        m_encoder.setPosition(0.0);
        Logger.logWithNetworkTable(table, "isRunning", false);
    }

    public void runTurret(double speed) {
        // if (Math.abs(speed) > Math.abs(TurretConstants.k_maxSpeed)) {
        // turretMotor.set(TurretConstants.k_maxSpeed * (speed / Math.abs(speed))); // limiting
        // speed & keeping pos or neg sign
        // } else {
        // turretMotor.set(speed);
        // }
        m_turretMotor.set(speed);
        Logger.logWithNetworkTable(table, "isRunning", true);
    }

    public double getCurrentPosition() {
        return m_encoder.getPosition();
    }

    public void setEncoderPosition(double position) {
        m_encoder.setPosition(position);
    }

    @Override
    public void periodic() {
        Logger.logWithNetworkTable(table, "Position", getCurrentPosition());
    }
}