// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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

        public static final boolean kOutreachMode = false;

        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int kRightDriveCAN1 = 1;
        public static final int kRightDriveCAN2 = 2;
        public static final int kRightDriveCAN3 = 3;
        public static final int kLeftDriveCAN1 = 4;
        public static final int kLeftDriveCAN2 = 5;
        public static final int kLeftDriveCAN3 = 6;

    }
    public static final class DriveConstants {
        public static final double kDriveSpeed = 0.6;
        public static final double kTurnSpeed = 0.3;
        public static final double kWheelDiameterInches = 6;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(kWheelDiameterInches);
        public static final double kGearRatio = 10.81;

        /** Joules per kilogram per meter squared */
        public static final double kJKgMetersSquared = 7.5;
        public static final double kRobotMass = 60;
        public static final double kTrackWidth = 0.7112;

        /** Static gain in volts */
        public static final double kS = .145;
        /** Velocity gain in volt seconds per meter */
        public static final double kV = 2.8;
        /** Acceleration gain in volt seconds squared per meter */
        public static final double kA = .425;

        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kDeadzone = 0;
    }
}
