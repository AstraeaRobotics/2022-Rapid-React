// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSubsystem extends SubsystemBase {

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  private int m_rainbowFirstPixelHue = 30; 


  public LEDSubsystem() {
    m_led = new AddressableLED(LEDConstants.kPwmPort); 
    m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kLength);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void glowRed() {
    for(int i =0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i,255,0,0);
    }
    // m_led.setData(m_ledBuffer);
  }

  public void glowGreen() {
    for (int i=0; i <m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i,0,255,0);
    }
    // m_led.setData(m_ledBuffer);
  }

  public void glowBlue() {
    for(int i =0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i,0,0,255);
    }
    // m_led.setData(m_ledBuffer);
  }

  public void rainbow() {
    for(int i = 0; i < m_ledBuffer.getLength(); i++) {
       int hue = (m_rainbowFirstPixelHue + (i*180/m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i,hue, 255, 150);
    }
    m_rainbowFirstPixelHue +=3;
    m_rainbowFirstPixelHue %=180;
  }

  public void flash(){
    for(int i =0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer = setRGB(255,255,0);
    }
        m_led.setData(m_ledBuffer);
        m_led.setSyncTime(1000000);
        // m_led.setData(m_ledBuffer); uncomment this 
        for(int i =0; i < m_ledBuffer.getLength(); i++){
          m_ledBuffer.setRGB(0,0,0);
        }
        m_led.setData(m_ledBuffer);
  }

  public void trailMixVersionTwo(){
    for(int i=0; i < m_ledBuffer.getLength(), i++){
      for(int t = 1; t < m_ledBuffer.getLength(); t++)
      if(t!=i){
        m_led.setRGB(t,0,255,0);
        m_led.setData(m_ledBuffer);
      }
      else{
        m_led.setRGB(t,0,0,0);
        m.led.setData(m_ledBuffer);
      }
    }
  }

  public void trailMixVersionTwoWithoutUsingAdviksMethod(){
    for(int = 0; i < m_ledBuffer.getLength(); i++){
      for(int = 0; n < (m_ledBuffer.getLength) -1; n++){
        if(n==i){
        m_led.setRGB(n,0,255,0);
        m_led.setData(m_ledBuffer);
        }
        else{
          m_led.setRGB(n,0,0,0);
          m_led.setData(m_ledBuffer);
        }
      }
    }
  }

  public void trailMixForKylieLiu() { //this will do Kyle's trail method 
     //int trailLength=5;
     for(int n=0; n< m_ledBuffer.getLength(); n++) {
      for(int i =0; i < m_ledBuffer.getLength(); i++) {
      if(i!=n) {
        m_led.setRGB(i,0,0,255); //background color 
        m_led.setData(m_ledBuffer);
      } else { //when i=n
        m_led.setRGB(i,0,255,0); //trailing color 
        m_led.setData(m_ledBuffer);
      }
      
      
      }
      m_led.setSyncTime(500000); //to delay between trail color moving 
     }
    
  }

  @Override
  public void periodic() {
    //rainbow();
    trailMixForKylieLiu();
    //flash();
   // m_led.setData(m_ledBuffer);
  }
}