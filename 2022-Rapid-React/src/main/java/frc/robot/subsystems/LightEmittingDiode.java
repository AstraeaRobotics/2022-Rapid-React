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


  AddressableLEDBuffer LightEmittingDiodeBuffer;
  AddressableLED LightEmittingDiode;

  int startingColor =30; 

  public LightEmittingDiode() {
    LightEmittingDiode = new AddressableLED(Constants.LightEmittingDiodeConstants.LightEmittingDiodePort); 

    LightEmittingDiodeBuffer = new AddressableLEDBuffer(144); //arbirtary length

    LightEmittingDiode.setLength(LightEmittingDiodeBuffer.getLength());
    LightEmittingDiode.setData(LightEmittingDiodeBuffer);


    LightEmittingDiode.start();

  
  }
/*
  public void glowGreen() { //this will make the Light emititing diode appear green 
    for(int i =0; i < LightEmittingDiodeBuffer.getLength(); i++) {
      LightEmittingDiodeBuffer.setHSV(i,150,100,100);
    }
  }
  */ 
  public void glowGreen() {
    for (int i=0; i <LightEmittingDiodeBuffer.getLength(); i++) {
      LightEmittingDiodeBuffer.setRGB(i,0,255,0);
    }
  }
  /*
  public void glowRed() { //this will make the Light emititing diode appear red 
    for(int i =0; i < LightEmittingDiodeBuffer.getLength(); i++)
      LightEmittingDiodeBuffer.setHSV(i,30,100,100);
  }
*/
/*
  public void glowBlue() { //this will make the Light emititing diode appear blue 
    for(int i =0; i < LightEmittingDiodeBuffer.getLength(); i++)
      LightEmittingDiodeBuffer.setHSV(i, 280, 100, 100);
  }
  */
  
  public void blink(int blinkcolor) { //flashes

  }


  public void rainbow() {

    for(int i = 0; i < LightEmittingDiodeBuffer.getLength(); i++) {
       int hue = (startingColor + (i*180/LightEmittingDiodeBuffer.getLength())) % 180;

      LightEmittingDiodeBuffer.setHSV(i,hue, 255, 150);
    }
    startingColor +=3;
    startingColor %=180;

  }


  @Override
  public void periodic() {
      // This method will be called once per scheduler run

    // rainbow();
    // glowGreen();

  }
}