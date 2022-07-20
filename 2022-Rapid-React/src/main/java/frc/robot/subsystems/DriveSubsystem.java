package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.status.Status;
import frc.robot.Robot;
import frc.robot.util.Logger;
import frc.robot.util.SparkMax;

public class DriveSubsystem extends SubsystemBase {

    AHRS gyro;

    CANSparkMax leftLeader;
    CANSparkMax leftFollower1;
    CANSparkMax leftFollower2;

    CANSparkMax rightLeader;
    CANSparkMax rightFollower1;
    CANSparkMax rightFollower2;

    MotorControllerGroup leftMotors;
    MotorControllerGroup rightMotors;

    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;
    RelativeEncoder leftEncoder2;
    RelativeEncoder rightEncoder2;
    RelativeEncoder leftEncoder3;
    RelativeEncoder rightEncoder3;

    DifferentialDrive m_drive;
    DifferentialDrivetrainSim m_driveSim;

    NetworkTable table;

    DifferentialDriveKinematics kinematics;
    DifferentialDriveOdometry odometry;

    SimpleMotorFeedforward feedforward;
    PIDController leftPID;
    PIDController rightPID;

    public Field2d field;
    Pose2d pose;

    SendableChooser<Integer> m_chooser;
    int simInvert;

    String status;

    public DriveSubsystem() {
        gyro = new AHRS(SPI.Port.kMXP);

        rightLeader = SparkMax.constructSparkMax(Constants.RobotMap.RIGHT_LEADER_CAN, true);
        rightFollower1 = SparkMax.constructSparkMax(Constants.RobotMap.RIGHT_FOLLOWER_CAN1, true);
        rightFollower2 = SparkMax.constructSparkMax(Constants.RobotMap.RIGHT_FOLLOWER_CAN2, true);

        leftLeader = SparkMax.constructSparkMax(Constants.RobotMap.LEFT_LEADER_CAN, true);
        leftFollower1 = SparkMax.constructSparkMax(Constants.RobotMap.LEFT_FOLLOWER_CAN1, true);
        leftFollower2 = SparkMax.constructSparkMax(Constants.RobotMap.LEFT_FOLLOWER_CAN2, true);

        leftEncoder = leftLeader.getEncoder();
        leftEncoder2 = leftFollower1.getEncoder();
        leftEncoder3 = leftFollower2.getEncoder();

        rightEncoder = rightLeader.getEncoder();
        rightEncoder2 = rightFollower1.getEncoder();
        rightEncoder3 = rightFollower2.getEncoder();

        leftEncoder.setPosition(0d);
        rightEncoder.setPosition(0d);
        leftEncoder2.setPosition(0d);
        rightEncoder2.setPosition(0d);
        leftEncoder3.setPosition(0d);
        rightEncoder3.setPosition(0d);

        leftEncoder.setPositionConversionFactor(
            (Units.inchesToMeters(Constants.DriveConstants.WHEEL_DIAMETER) * Math.PI) / Constants.DriveConstants.GEAR_RATIO
        );
        rightEncoder.setPositionConversionFactor(
            (Units.inchesToMeters(Constants.DriveConstants.WHEEL_DIAMETER) * Math.PI) / Constants.DriveConstants.GEAR_RATIO
        );
        leftEncoder2.setPositionConversionFactor(
            (Units.inchesToMeters(Constants.DriveConstants.WHEEL_DIAMETER) * Math.PI) / Constants.DriveConstants.GEAR_RATIO
        );
        rightEncoder2.setPositionConversionFactor(
            (Units.inchesToMeters(Constants.DriveConstants.WHEEL_DIAMETER) * Math.PI) / Constants.DriveConstants.GEAR_RATIO
        );
        leftEncoder3.setPositionConversionFactor(
            (Units.inchesToMeters(Constants.DriveConstants.WHEEL_DIAMETER) * Math.PI) / Constants.DriveConstants.GEAR_RATIO
        );
        rightEncoder3.setPositionConversionFactor(
            (Units.inchesToMeters(Constants.DriveConstants.WHEEL_DIAMETER) * Math.PI) / Constants.DriveConstants.GEAR_RATIO
        );

        leftMotors = new MotorControllerGroup(leftLeader, leftFollower1, leftFollower2);
        rightMotors = new MotorControllerGroup(rightLeader, rightFollower1, rightFollower2);

        m_drive = new DifferentialDrive(leftMotors, rightMotors);

        field = new Field2d();

        table = NetworkTableInstance.getDefault().getTable("Drive");

        leftMotors.setInverted(false);
        rightMotors.setInverted(true);
        setBrake(true);

        m_driveSim =
            new DifferentialDrivetrainSim(
                DCMotor.getNEO(3),
                Constants.DriveConstants.GEAR_RATIO,
                Constants.DriveConstants.jKg_METERS_SQUARED,
                DriveConstants.ROBOT_MASS,
                Units.inchesToMeters(DriveConstants.WHEEL_DIAMETER / 2),
                DriveConstants.TRACK_WIDTH,
                null
            );

        kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.TRACK_WIDTH);
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        feedforward =
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter
            );
          
        leftPID = new PIDController(Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD);
        rightPID = new PIDController(Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD);

        simInvert = Robot.isReal() ? -1 : 1;

        feedforward =
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter
            );

        leftPID = new PIDController(Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD);
        rightPID = new PIDController(Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD);

        ShuffleboardTab driveTab = Shuffleboard.getTab("Dashboard");
        driveTab.add("Gyro", gyro).withWidget(BuiltInWidgets.kGyro);
        driveTab.add("Field View", field).withWidget("Field");

        
    }

    /**
     * Gets the motors in the subsystem
     *
     * @return the motors
     */
    public CANSparkMax[] getMotors() {
        return new CANSparkMax[] { leftLeader, leftFollower1, leftFollower2, rightLeader, rightFollower1, rightFollower2 };
    }

    /**
     * Sets the default brake mode for the drivetrain.
     *
     * @param brake Whether or not to use brake mode.
     */
    public void setBrake(boolean on) {
        IdleMode mode = on ? IdleMode.kBrake : IdleMode.kCoast;
        CANSparkMax[] motors = getMotors();
        for (CANSparkMax motor : motors) {
            motor.setIdleMode(mode);
        }
    }

    /**
     * Controls each side of the robot individually.
     *
     * @param left  The speed of the left side of the robot.
     * @param right The speed of the right side of the robot.
     */
    public void tankDrive(double left, double right) {
        m_drive.tankDrive(left * DriveConstants.DRIVE_SPEED, right * DriveConstants.DRIVE_SPEED);
    }

    public void tan(double speed) {
        m_drive.tankDrive(speed * Constants.DriveConstants.TURN_SPEED, speed * Constants.DriveConstants.TURN_SPEED);
    }

    public void tankDriveAuto(double left, double right) {
      m_drive.tankDrive(left, right);
  }

    /**
     * Controls the robot with curveDrive.
     *
     * @param xSpeed   The robot's speed along the X axis [-1.0..1.0]. Forward is
     *                 positive.
     * @param rotation The robot's rotation rate around the Z axis [-1.0..1.0].
     *                 Clockwise is positive.
     * @param turn     Whether or not to turn in place.
     */
    public void curveDrive(double xSpeed, double rotation, boolean turn) {
        m_drive.curvatureDrive(xSpeed * DriveConstants.DRIVE_SPEED, rotation * DriveConstants.TURN_SPEED, turn);
    }

    public void arcadeDrive(double xSpeed, double rotation, boolean stabilize) {
        m_drive.arcadeDrive(xSpeed * DriveConstants.DRIVE_SPEED, rotation * DriveConstants.TURN_SPEED, stabilize);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        m_drive.feed();
    }

    /** Resets the NavX to 0 degrees. */
    public void resetGyro() {
        gyro.zeroYaw();
    }

    /**
     * Returns the current heading of the robot in degrees.
     *
     * @return The current heading of the robot in degrees.
     */
    public double getHeading() {
        return gyro.getAngle();
    }

    @Override
    public void periodic() {
        pose = odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition() * simInvert);
        field.setRobotPose(pose);
        log();
    }

    @Override
    public void simulationPeriodic() {
        m_driveSim.setInputs(
            // Left and right CAN IDs are flipped on the robot so right and left sides are
            // flipped for
            // simulation
            rightMotors.get() * RobotController.getInputVoltage(),
            leftMotors.get() * RobotController.getInputVoltage()
        );
        m_driveSim.update(0.02);

        leftEncoder.setPosition(m_driveSim.getLeftPositionMeters());
        rightEncoder.setPosition(m_driveSim.getRightPositionMeters());

        int leftHandle = SimDeviceDataJNI.getSimDeviceHandle("SPARK MAX [" + leftLeader.getDeviceId() + "]");
        int rightHandle = SimDeviceDataJNI.getSimDeviceHandle("SPARK MAX [" + rightLeader.getDeviceId() + "]");
        SimDouble leftVelocity = new SimDouble(SimDeviceDataJNI.getSimValueHandle(leftHandle, "Velocity"));
        SimDouble rightVelocity = new SimDouble(SimDeviceDataJNI.getSimValueHandle(rightHandle, "Velocity"));
        leftVelocity.set(m_driveSim.getLeftVelocityMetersPerSecond());
        rightVelocity.set(m_driveSim.getRightVelocityMetersPerSecond());

        int gyroHandle = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(gyroHandle, "Yaw"));
        angle.set(m_driveSim.getHeading().getDegrees());
    }

    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }

    public PIDController getLeftPIDController() {
        return leftPID;
    }

    public PIDController getRightPIDController() {
        return rightPID;
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Resets encoder positions to 0
     */
    public void resetEncoders() {
        leftLeader.getEncoder().setPosition(0.0);
        rightLeader.getEncoder().setPosition(0.0);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    /**
     * Sets drive motors in volts
     * 
     * @param leftVolts Left motor in volts
     * @param rightVolts Right motor in volts
     */
    public void setOutput(double leftVolts, double rightVolts) {
        leftMotors.set(leftVolts / 12);
        rightMotors.set(rightVolts / 12);
    }

    public double getEncoderPosition() {
        return leftEncoder.getPosition();
    }

    public Pose2d getPose() {
        return pose;
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            leftEncoder.getVelocity() / Constants.DriveConstants.GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(3.0) / 60,
            rightEncoder.getVelocity() / Constants.DriveConstants.GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(3.0) / 60
        );
    }

    public void log() {
        Logger.logWithNetworkTable(table, "Heading", getHeading());
        Logger.logWithNetworkTable(table, "L1 Vel", leftEncoder.getVelocity());
        Logger.logWithNetworkTable(table, "L2 Vel", leftEncoder2.getVelocity());
        Logger.logWithNetworkTable(table, "L3 Vel", leftEncoder3.getVelocity());
        Logger.logWithNetworkTable(table, "R1 Vel", rightEncoder.getVelocity());
        Logger.logWithNetworkTable(table, "R2 Vel", rightEncoder2.getVelocity());
        Logger.logWithNetworkTable(table, "R3 Vel", rightEncoder3.getVelocity());

        Status.logStatus("Drive/Status", "Operational");
    }
}