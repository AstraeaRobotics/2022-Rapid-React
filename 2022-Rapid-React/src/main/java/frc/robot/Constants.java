// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
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
        public static final double kTurnSpeed = 1;
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
        public static final double kDeadzone = .2;
    }

    public static final class LEDConstants {
        public static final int kPwmPort = 0; 
        public static final int kLength = 144;
        public static final double kInterval = Units.secondsToMilliseconds(1.0);
        public static final double kIntervalTrail = Units.secondsToMilliseconds(0.1);

    }

    public static final class StatusConstants {
        public static final String kIndexer = "indexer";
        public static final String kShooter = "shooter";
        public static final String kIntake = "intake";
    }
    public static final class TurretConstants {
        public static final int kTurretCANId = 20;
        public static final double kVisionThreshold = 5;

        public static final double kMaxSpeed = 0.05;
    }
    public static final class Limelight {
        public static final double kLimeLightMountAngleDegrees = 25.0;
        public static final double kLimelightLensHeightInches = 20.0;
        public static final double kGoalHeightInches = 60.0;
    }

    public static final class Shooter {
        public static final int kPIDLoopIDx = 0;
        public static final double kGains_VelocitkF = 0.048973143759; //Feed Forward
        public static final double kGains_VelocitkP = 0.085035;
        public static final double kGains_VelocitkI = 0.00035;
        public static final double kGains_VelocitkD = 0;
        public static final double kConversionFactor = 2048/600;
        public static final int kTimeoutMs = 0;
        
        public static final int kMaxSpeed = 20000;

        public static final double kFlywheelA = 0;
        public static final double kFlywheelB = 1;
        public static final double kFlywheelC = 0;

        public static final double kFeederA = 0;
        public static final double kFeederB = 1;
        public static final double kFeederC = 0;

    }
}
