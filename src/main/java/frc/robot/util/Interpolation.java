// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;
import java.util.NavigableMap;
import java.util.TreeMap;

import edu.wpi.first.hal.util.BoundaryException;
import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class Interpolation {
    private NavigableMap<Double, Double> m_map;
    
    public Interpolation() {
        m_map = new TreeMap<Double, Double>();
    }

    public Interpolation(NavigableMap<Double, Double> map) {
        m_map = map;
    }

    /*
     * @return true if map is empty
     */
    public boolean checkMap() {
        if (m_map == null || m_map.size() == 0) {
            return true;
        }
        return false;
    }

    public void addPoint(double ty, double speed) {
        if (m_map.get(ty) != null) {
            return;
        }
        m_map.put(ty, speed);
    }

    public double getNearest(double target_ty) throws IndexOutOfBoundsException {
        if (checkMap()) { // TODO: FIX ME - get more accurate exception
            throw new IndexOutOfBoundsException("Map is empty");
        }
        if (m_map.get(target_ty) != null) {
            return m_map.get(target_ty);
        }

        var lower = m_map.lowerEntry(target_ty);
        var upper = m_map.higherEntry(target_ty);

        if (lower == null) {
            return upper.getKey();
        }
        if (upper == null) {
            return lower.getKey();
        }

        //Return the key cloeset to the target key
        return Math.abs(target_ty-lower.getKey()) <= Math.abs(target_ty-upper.getKey()) ? lower.getKey() : upper.getKey();
    }

    public double interpolate(double target_ty) throws BoundaryException {
        if (checkMap()) {
            throw new BoundaryException("Map is empty. There. Is. No. Bounds.");
        }
        
        if (m_map.get(target_ty) != null) {
            return m_map.get(target_ty);
        }
        
        //Get lowest and highest values
        var lower = m_map.lowerEntry(target_ty);
        var upper = m_map.higherEntry(target_ty);

        if (lower == null || upper == null) {
            throw new BoundaryException("Target ty is out of bounds");
        }

        //Get the distances
        double distKeyUpper = upper.getKey() - target_ty;
        double distKeyLower = target_ty - lower.getKey();
        double distanceBetween = upper.getKey() - lower.getKey();

        //t is from 0 to 1, which is the percentage distance between two numbers upper and lower
        double t = distKeyLower <= distKeyUpper ? distKeyLower / distanceBetween : 1 - distKeyUpper / distanceBetween;

        return MathUtil.interpolate(lower.getValue(), upper.getValue(), t);
    }
}
