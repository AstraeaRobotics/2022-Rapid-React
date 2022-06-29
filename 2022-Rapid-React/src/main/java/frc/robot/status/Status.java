package frc.robot.status;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Status {

    // STATUSES
    // drivebase: auto, forward, backward
    // elevator: bottom, going_up, going_down, top
    // intake: extended_spinning, retracted_stopped,
    // shooter: ready, shooting, stopped
    // indexer: no_balls, one_ball, two_balls

    
    public static void logStatus(String system, String status) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("status");
        table.getEntry(system).setString(status);
    }

    public static String getStatus(String system) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("status");
        String status = table.getEntry(system).getString("");
        return status;
    }
    
}
