// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

//INIT or Global Vars

public class VisionSubsystem extends SubsystemBase {
  /* Creates a new VisionSubsystem. */
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = table.getEntry("tv"); //Verify if this works
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public double getXoffset() {
    return tx.getDouble(0.0);
  }

  public double getYOffset() {
    return ty.getDouble(0.0);
  }
  
  public double getTargetArea() {
    return ta.getDouble(0.0);
  }

  public boolean getTargetStatus() {
    double value = table.getEntry("tv").getDouble(0);
    if (value == 0) {
        return false;
    } else if (value == 1) {
        return true;
    }
    SmartDashboard.putString("LIMELIGHT:", "ERROR: Target not found. Check Limelight connection.");
    return false;
  }

  public void setLED(boolean on) {
    table.getEntry("ledMode").setNumber(on ? 3 : 1);
  }

  public double getDistanceToTarget() {
    double yOffset = getYOffset();

    double angleToGoalDegrees = Constants.VisionSubsystem.kLimeLightMountAngleDegrees + yOffset;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

    //calculate distance
    return (Constants.VisionSubsystem.kGoalHeightInches - Constants.VisionSubsystem.kLimelightLensHeightInches)/Math.tan(angleToGoalRadians);
  }
}
