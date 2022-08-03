// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class Regression {

    /**
     * Returns value of binomial regression function with the form ax^2 + bx + c
     * @param a coefficient for the x^2 term
     * @param b coefficient for the x term
     * @param c constant
     * @param x input value
     * @return the value of the function
     */
    public static double binomialRegression(double a, double b, double c, double x) {
        return a*Math.pow(x, 2) + b*x + c;
    }
}
