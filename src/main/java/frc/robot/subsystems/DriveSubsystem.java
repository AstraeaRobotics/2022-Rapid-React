package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.status.Status;
import frc.robot.util.Logger;

public class DriveSubsystem extends SubsystemBase {

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    private CANSparkMax m_leftMotor1 = new CANSparkMax(RobotMap.kLeftDriveCAN1, MotorType.kBrushless);
    private CANSparkMax m_leftMotor2 = new CANSparkMax(RobotMap.kLeftDriveCAN2, MotorType.kBrushless);
    private CANSparkMax m_leftMotor3 = new CANSparkMax(RobotMap.kLeftDriveCAN3, MotorType.kBrushless);

    private CANSparkMax m_rightMotor1 = new CANSparkMax(RobotMap.kRightDriveCAN1, MotorType.kBrushless);
    private CANSparkMax m_rightMotor2 = new CANSparkMax(RobotMap.kRightDriveCAN2, MotorType.kBrushless);
    private CANSparkMax m_rightMotor3 = new CANSparkMax(RobotMap.kRightDriveCAN3, MotorType.kBrushless);

    private MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2, m_leftMotor3);
    private MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotor1, m_rightMotor2, m_rightMotor3);

    private RelativeEncoder m_leftEncoder1 = m_leftMotor1.getEncoder();
    private RelativeEncoder m_leftEncoder2 = m_leftMotor2.getEncoder();
    private RelativeEncoder m_leftEncoder3 = m_leftMotor3.getEncoder();

    private RelativeEncoder m_rightEncoder1 = m_rightMotor1.getEncoder();
    private RelativeEncoder m_rightEncoder2 = m_rightMotor2.getEncoder();
    private RelativeEncoder m_rightEncoder3 = m_rightMotor3.getEncoder();

    private DifferentialDrive m_drive;
    private DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(3),
            DriveConstants.kGearRatio,
            DriveConstants.kJKgMetersSquared,
            DriveConstants.kRobotMass,
            Units.inchesToMeters(DriveConstants.kWheelDiameterInches / 2),
            DriveConstants.kTrackWidth,
            null);

    private NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Drive");

    private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidth);
    private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
            Constants.DriveConstants.kS,
            Constants.DriveConstants.kV,
            Constants.DriveConstants.kA);
    private PIDController m_leftPID = new PIDController(
            Constants.DriveConstants.kP,
            Constants.DriveConstants.kI,
            Constants.DriveConstants.kD);
    private PIDController m_rightPID = new PIDController(
            Constants.DriveConstants.kP,
            Constants.DriveConstants.kI,
            Constants.DriveConstants.kD);

    private Field2d m_field = new Field2d();
    private Pose2d m_pose;

    String status;

    public DriveSubsystem() {

        m_leftEncoder1.setPosition(0d);
        m_rightEncoder1.setPosition(0d);
        m_leftEncoder2.setPosition(0d);
        m_rightEncoder2.setPosition(0d);
        m_leftEncoder3.setPosition(0d);
        m_rightEncoder3.setPosition(0d);

        m_leftEncoder1.setPositionConversionFactor(
                (DriveConstants.kWheelDiameterMeters * Math.PI) / DriveConstants.kGearRatio);
        m_rightEncoder1.setPositionConversionFactor(
                (DriveConstants.kWheelDiameterMeters * Math.PI) / DriveConstants.kGearRatio);
        m_leftEncoder2.setPositionConversionFactor(
                (DriveConstants.kWheelDiameterMeters * Math.PI) / DriveConstants.kGearRatio);
        m_rightEncoder2.setPositionConversionFactor(
                (DriveConstants.kWheelDiameterMeters * Math.PI) / DriveConstants.kGearRatio);
        m_leftEncoder3.setPositionConversionFactor(
                (DriveConstants.kWheelDiameterMeters * Math.PI) / DriveConstants.kGearRatio);
        m_rightEncoder3.setPositionConversionFactor(
                (DriveConstants.kWheelDiameterMeters * Math.PI) / DriveConstants.kGearRatio);

        m_leftMotors.setInverted(false);
        m_rightMotors.setInverted(true);
        setBrake(true);

        // m_drive must be initialized here because motors must be inverted first
        m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

        ShuffleboardTab driveTab = Shuffleboard.getTab("Dashboard");
        driveTab.add("Gyro", gyro).withWidget(BuiltInWidgets.kGyro);
        driveTab.add("Field View", m_field).withWidget("Field");

    }

    /**
     * Gets the motors in the subsystem
     *
     * @return the motors
     */
    public CANSparkMax[] getMotors() {
        return new CANSparkMax[] { m_leftMotor1, m_leftMotor2, m_leftMotor3, m_rightMotor1, m_rightMotor2,
                m_rightMotor3 };
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
        m_drive.tankDrive(left * DriveConstants.kDriveSpeed, right * DriveConstants.kDriveSpeed);
    }

    /**
     * Spins both sides of the robot at the same speed
     * 
     * @param speed The speed to spin both wheels
     */
    public void tankDrive(double speed) {
        m_drive.tankDrive(speed * Constants.DriveConstants.kTurnSpeed, speed * Constants.DriveConstants.kTurnSpeed);
    }

    /**
     * Controls each side of the robot individually. This method does not scale
     * input values and passes them directly into the drive controller
     * 
     * @param left  The speed of the left side of the robot
     * @param right The speed of the right side of the robot
     */
    public void tankDriveRaw(double left, double right) {
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
        m_drive.curvatureDrive(xSpeed * DriveConstants.kDriveSpeed, rotation * DriveConstants.kTurnSpeed, turn);
    }

    public void arcadeDrive(double xSpeed, double rotation, boolean stabilize) {
        m_drive.arcadeDrive(xSpeed * DriveConstants.kDriveSpeed, rotation * DriveConstants.kTurnSpeed, stabilize);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(rightVolts);
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
        m_pose = m_odometry.update(gyro.getRotation2d(), m_leftEncoder1.getPosition(),
                m_rightEncoder1.getPosition());
        m_field.setRobotPose(m_pose);
        log();
    }

    @Override
    public void simulationPeriodic() {
        m_driveSim.setInputs(
                // Left and right CAN IDs are flipped on the robot so right and left sides are
                // flipped for
                // simulation
                m_rightMotors.get() * RobotController.getInputVoltage(),
                m_leftMotors.get() * RobotController.getInputVoltage());
        m_driveSim.update(0.02);

        m_leftEncoder1.setPosition(m_driveSim.getLeftPositionMeters());
        m_rightEncoder1.setPosition(m_driveSim.getRightPositionMeters());

        int leftHandle = SimDeviceDataJNI.getSimDeviceHandle("SPARK MAX [" + m_leftMotor1.getDeviceId() + "]");
        int rightHandle = SimDeviceDataJNI.getSimDeviceHandle("SPARK MAX [" + m_rightMotor1.getDeviceId() + "]");
        SimDouble leftVelocity = new SimDouble(SimDeviceDataJNI.getSimValueHandle(leftHandle, "Velocity"));
        SimDouble rightVelocity = new SimDouble(SimDeviceDataJNI.getSimValueHandle(rightHandle, "Velocity"));
        leftVelocity.set(m_driveSim.getLeftVelocityMetersPerSecond());
        rightVelocity.set(m_driveSim.getRightVelocityMetersPerSecond());

        int gyroHandle = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(gyroHandle, "Yaw"));
        angle.set(m_driveSim.getHeading().getDegrees());
    }

    public SimpleMotorFeedforward getFeedforward() {
        return m_feedforward;
    }

    public PIDController getLeftPIDController() {
        return m_leftPID;
    }

    public PIDController getRightPIDController() {
        return m_rightPID;
    }

    public DifferentialDriveKinematics getKinematics() {
        return m_kinematics;
    }

    /**
     * Resets encoder positions to 0
     */
    public void resetEncoders() {
        m_leftMotor1.getEncoder().setPosition(0.0);
        m_rightMotor1.getEncoder().setPosition(0.0);
    }

    /**
     * Resets odometry to base position.
     * 
     * NOTE: This method also resets encoders.
     * 
     * @param pose The pose to set the robot to
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, gyro.getRotation2d());
    }

    /**
     * Sets drive motors in volts
     * 
     * @param leftVolts  Left motor in volts
     * @param rightVolts Right motor in volts
     */
    public void setOutput(double leftVolts, double rightVolts) {
        m_leftMotors.set(leftVolts / 12);
        m_rightMotors.set(rightVolts / 12);
    }

    /**
     * Gets the tracked encoder position of the left front encoder
     * 
     * @return the tracked encoder position
     */
    public double getEncoderPosition() {
        return m_leftEncoder1.getPosition();
    }

    /**
     * Returns the tracked pose of the robot
     * 
     * @return the robot's pose
     */
    public Pose2d getPose() {
        return m_pose;
    }

    /**
     * Gets the wheel speeds of the robot's wheels
     * 
     * @return The differential drive wheel speeds
     */
    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                m_leftEncoder1.getVelocity() / Constants.DriveConstants.kGearRatio * 2 * Math.PI
                        * Units.inchesToMeters(3.0) / 60,
                m_rightEncoder1.getVelocity() / Constants.DriveConstants.kGearRatio * 2 * Math.PI
                        * Units.inchesToMeters(3.0) / 60);
    }

    /**
     * Logs important data for the drivebase subsystem
     */
    public void log() {
        Logger.logWithNetworkTable(m_table, "Heading", getHeading());
        Logger.logWithNetworkTable(m_table, "L1 Vel", m_leftEncoder1.getVelocity());
        Logger.logWithNetworkTable(m_table, "L2 Vel", m_leftEncoder2.getVelocity());
        Logger.logWithNetworkTable(m_table, "L3 Vel", m_leftEncoder3.getVelocity());
        Logger.logWithNetworkTable(m_table, "R1 Vel", m_rightEncoder1.getVelocity());
        Logger.logWithNetworkTable(m_table, "R2 Vel", m_rightEncoder2.getVelocity());
        Logger.logWithNetworkTable(m_table, "R3 Vel", m_rightEncoder3.getVelocity());

        Status.logStatus("Drive/Status", "Operational");
    }
}