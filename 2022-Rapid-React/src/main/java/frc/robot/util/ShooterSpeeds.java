// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.Constants;

/** Add your docs here. */
public class ShooterSpeeds {
    double[] speedVals = new double[2];

    public ShooterSpeeds(double topSpeed, double bottomSpeed) {
        speedVals[0] = topSpeed;
        speedVals[1] = bottomSpeed;
    }

    public double getTopVelocity() {
        return speedVals[0];
    }

    public double getBottomVelocity() {
        return speedVals[1];
    }

    public void setTopVelocity(double speed) {
        speedVals[0] = speed;
    }

    public void setBottomVelocity(double speed) {
        speedVals[1] = speed;
    }

    /*
     * Assuming that 20k is the top velocity
     * Have methods that returns speeds for each percent
     * function (int x (0-100)) return speed
     * 
     */
    public static double getSpeedPercent(double percent) {
        return Constants.Shooter.kMaxSpeed * percent / 100;
    }
}
