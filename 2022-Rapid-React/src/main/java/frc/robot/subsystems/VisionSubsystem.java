// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

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


  public void getXoffset() {
    //Reads values periodically
    double x = tx.getDouble(0.0);
  }

  public void getYOffset() {
    //Reads values
    double y = ty.getDouble(0.0);
  }
  
  public void getTargetArea() {
    double area = ta.getDouble(0.0);
  }
}
