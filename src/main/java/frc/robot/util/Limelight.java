package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants;

public class Limelight {

    /**
     *  Get horizontal offset from center of target
     *  @return horizontal offset from center of target
     */

    public static double getTx() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    /**
     *  Get vertical offset from center of target
     *  @return vertical offset from center of target
     */
    public static double getTy() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }

    /**
     *  Get target area --> Target Area (0% of image to 100% of image)
     *  @return target area 
     */
    public static double getTa() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    }

    /**
     * Get whether or not there is a target in sight (0 = no target, 1 = target)
     * @return whether or not there is a target in frame
     */
    public static boolean getTv() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false);
    }

    /**
     * Set the LED mode (0 = current pipeline mode, 1 = force off, 2 = force blink, 3 = force on)
     * @param mode
     */
    public static void setLEDMode(int mode) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode);
    }

    public double getDistanceToTarget() {
        double yOffset = getTy();
        double angleToGoalDegrees = Constants.Limelight.kLimeLightMountAngleDegrees + yOffset;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
        //calculate distance
        return (Constants.Limelight.kGoalHeightInches - Constants.Limelight.kLimelightLensHeightInches)/Math.tan(angleToGoalRadians);
    }

    /**
     * Set the pipeline number to use
     * @param pipeline
     */
    public static void setPipeline(int pipeline) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    }

}
