package frc.robot.status;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.StatusConstants;

public class Status {
    
    public enum ShooterStatus {
        kReady,
        kStopped
    }
    
    public enum IntakeStatus {
        kExtended,
        kRetracted
    }

    public enum DriveStatus {
        kForward,
        kBackward,
        kStopped
    }    

    public enum BallStatus {
        kEmpty,
        kBlue,
        kRed
    }
    
    public enum ClimberStatus {
        kClimbing, 
        kStopped
    }

    public enum IndexerStatus {
        kLoading,
        kShooting,
        kStopped
    }

    /* DEFAULT STATUS */

    public static void logStatus(String system, String status) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(system);
        table.getEntry("status").setString(status);
    }

    public static void logStatus(String system, String status, String key) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(system);
        table.getEntry(key).setString(status);
    }

    public static String checkStatus(String system, String defaultValue) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(system);
        return table.getEntry("status").getString(defaultValue);
    }

    public static String checkStatus(String system, String defaultValue, String key) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(system);
        return table.getEntry(key).getString(defaultValue);
    }

    /* SHOOTER */

    public static void logShooterStatus(ShooterStatus status) {
        logStatus(StatusConstants.kShooter, status.name());
    }

    public static ShooterStatus getShooterStatus() {
        return ShooterStatus.valueOf(checkStatus(StatusConstants.kShooter, "kStopped"));
    }

    /* INTAKE */

    public static void logIntakeStatus(IntakeStatus status) {
        logStatus(StatusConstants.kIntake, status.name());
    }

    public static IntakeStatus getIntakeStatus() {
        return IntakeStatus.valueOf(checkStatus(StatusConstants.kIntake, "kRetracted"));
    }

    /* INDEXER */

    public static void logBallStatus(int ballNumber, BallStatus status) {
        logStatus(StatusConstants.kIndexer, status.name(), "ball " + ballNumber);
    }

    //0 will always be the lower ball
    public static BallStatus getBallStatus(int ballNumber) {
        return BallStatus.valueOf(checkStatus(StatusConstants.kIndexer, "kEmpty", "ball " + ballNumber));
    }

    public static void logIndexerStatus(IndexerStatus status) {
        logStatus(StatusConstants.kIndexer, status.name());
    }

    public static IndexerStatus getIndexerStatus() {
        return IndexerStatus.valueOf(checkStatus(StatusConstants.kIndexer, "kStopped"));
    }

    /* CLIMBER */
    public static void logClimbStatus(ClimberStatus status) {
        logStatus(StatusConstants.kClimber, status.name());
    }

    public static ClimberStatus getClimbStatus() {
        return ClimberStatus.valueOf(checkStatus(StatusConstants.kClimber, "kStopped"));
    }
}
