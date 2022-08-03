// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;
import java.util.TreeMap;
import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class Interpolation {
    private static TreeMap<Double, Double> m_map = new TreeMap<>();
    
    public Interpolation(TreeMap<Double, Double> map) {
        m_map = map;
    }

    public void addPoint(double ty, double speed) {
        m_map.put(ty, speed);
    }

    public double getNearest(double target_ty) {
        if (m_map.get(target_ty) != null) {
            return m_map.get(target_ty);
        }

        var lower = m_map.lowerEntry(target_ty).getValue();
        var upper = m_map.higherEntry(target_ty).getValue();

        return lower < upper ? lower : upper;
    }

    public double interpolate(double target_ty) {
        if (m_map.get(target_ty) != null) {
            return m_map.get(target_ty);
        }
        
        //Get lowest and highest values
        var lower = m_map.lowerEntry(target_ty);
        var upper = m_map.higherEntry(target_ty);

        //Get the distances
        double distKeyUpper = upper.getKey() - target_ty;
        double distKeyLower = target_ty - lower.getKey();
        double distanceBetween = upper.getKey() - lower.getKey();

        //Get speeds such that they don't exceed the max speed
        double t = distKeyLower <= distKeyUpper ? distKeyLower / distanceBetween : 1 - distKeyUpper / distanceBetween;

        return MathUtil.interpolate(lower.getValue(), upper.getValue(), t);
    }
}
