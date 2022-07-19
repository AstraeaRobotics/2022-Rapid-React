// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class RobotMap {

        public static final boolean OUTREACH_MODE = false;

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final int RIGHT_LEADER_CAN = 1;
        public static final int RIGHT_FOLLOWER_CAN1 = 2;
        public static final int RIGHT_FOLLOWER_CAN2 = 3;
        public static final int LEFT_LEADER_CAN = 4;
        public static final int LEFT_FOLLOWER_CAN1 = 5;
        public static final int LEFT_FOLLOWER_CAN2 = 6;

    }
    public static final class DriveConstants {
        public static final double DRIVE_SPEED = 0.6;
        public static final double TURN_SPEED = 0.3;
        public static final double WHEEL_DIAMETER = 6; // in inches
        public static final double GEAR_RATIO = 10.81;

        public static final double jKg_METERS_SQUARED = 7.5;
        public static final double ROBOT_MASS = 60;
        public static final double TRACK_WIDTH = 0.7112;

        public static final double ksVolts = .145;
        public static final double kvVoltSecondsPerMeter = 2.8;
        public static final double kaVoltSecondsSquaredPerMeter = .425;

        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double DEADZONE = 0;
    }

    public static final class VisionSubsystem {
        public static final double kLimeLightMountAngleDegrees = 25.0;
        public static final double kLimelightLensHeightInches = 20.0;
        public static final double kGoalHeightInches = 60.0;

    }
}
