// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;



public class LightEmittingDiode extends SubsystemBase {
  /** Creates a new LightEmittingDiode. */
  public LightEmittingDiode() {
    AddressableLED LightEmittingDiode = new AddressableLED(Constants.LightEmittingDiodeConstants.LightEmittingDiodePort); 

    AddressableLEDBuffer LightEmittingDiodeBuffer = new AddressableLEDBuffer(0); //arbirtary length

    LightEmittingDiode.setLength(LightEmittingDiodeBuffer.getLength());
    LightEmittingDiode.setData(LightEmittingDiodeBuffer);

    LightEmittingDiode.start();
  }

  @Override
  public void periodic() {
  9
    // This method will be called once per scheduler run
  }
}