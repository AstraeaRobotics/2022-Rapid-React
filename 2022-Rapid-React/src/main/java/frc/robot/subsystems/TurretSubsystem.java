package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.util.Limelight;

import com.revrobotics.RelativeEncoder;

public class TurretSubsystem extends SubsystemBase {

    private CANSparkMax turretMotor;

    private final double maxTurretSpeed = Constants.TurretConstants.maxSpeed;

    private final RelativeEncoder turretMEnc;

    public TurretSubsystem() {
        turretMotor = new CANSparkMax(Constants.TurretConstants.TurretCANid, CANSparkMaxLowLevel.MotorType.kBrushless);
        turretMEnc = turretMotor.getEncoder();
    }

    public void runTurret(double speed) {
        // if (Math.abs(speed) > Math.abs(maxTurretSpeed)) {
        // turretMotor.set(maxTurretSpeed * (speed / Math.abs(speed))); // limiting
        // speed & keeping pos or neg sign
        // } else {
        // turretMotor.set(speed);
        // }
        turretMotor.set(speed);
    }

    public double getCurrentPosition() {
        return turretMEnc.getPosition();
    }

    public void setEncoderPosition(double position) {
        turretMEnc.setPosition(position);
    }
}