package frc.robot.util;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;

public class Traj {

    public static Trajectory createNewTrajectoryFromJSON(String filename) {
        Path path = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/" + filename + ".wpilib.json");
        try {
            return TrajectoryUtil.fromPathweaverJson(path);
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }
    
}
