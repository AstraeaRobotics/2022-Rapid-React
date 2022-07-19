// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;



public class LightEmittingDiode extends SubsystemBase {
  /** Creates a new LightEmittingDiode. */
  public LightEmittingDiode() {
    AddressableLED LightEmittingDiode = new AddressableLED(3); //this port cannot be changed

    AddressableLEDBuffer LightEmittingDiodeBuffer = new AddressableLEDBuffer(9); //arbirtary length

    LightEmittingDiode.setLength(LightEmittingDiodeBuffer.getLength());
    LightEmittingDiode.setData(LightEmittingDiodeBuffer);
    
    LightEmittingDiode.start();
  }

  @Override
  public void periodic() {
  
    // This method will be called once per scheduler run
  }
}