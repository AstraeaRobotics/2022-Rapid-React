/********************************************************************************
 * * Copyright (c) Astraea Robotics, FIRST, and other WPILib contributors * * Open Source Software;
 * you can modify and/or share it under the terms of * the license file in the root directory of
 * this project. * *
 ********************************************************************************/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.status.Status;
import frc.robot.status.Status.IndexerStatus;
import frc.robot.status.Status.IntakeStatus;

public class LEDSubsystem extends SubsystemBase {


  boolean enabled = false;

  private boolean isOn = true;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private final AddressableLEDSim m_ledSim;

  private long m_previousTimeFlash = 0;

  private int m_rainbowFirstPixelHue = 30;
  private int currentTrailIndex = 0;

  public LEDSubsystem() {
    m_led = new AddressableLED(1);
    m_ledBuffer = new AddressableLEDBuffer(288);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    m_ledSim = new AddressableLEDSim(m_led);
    Shuffleboard.getTab("Dashboard").addRaw("Addressable Led", this::getLedData).withWidget("Addressable LED");
  }

  private byte[] getLedData() {
    return m_ledSim.getData();
  }

  public void setIsEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  public void glow(int r, int g, int b) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
  }

  public void glow(Color color) {
    glow((int) color.red, (int) color.green, (int) color.blue);
  }

  public void rainbow() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      int hue =
          (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    m_rainbowFirstPixelHue += 2;
    m_rainbowFirstPixelHue %= 180;
  }

  public void flash(int r, int g, int b) {
    long currentTime = System.currentTimeMillis();
    if ((currentTime - m_previousTimeFlash) >= LEDConstants.kInterval) {
      m_previousTimeFlash = currentTime;
      isOn = !isOn;
    }
    if (!isOn) {
      r = 0;
      g = 0;
      b = 0;
    }
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
  }

  public void flash(Color color) {
    flash((int) color.red, (int) color.green, (int) color.blue);
  }

  public void trail(Color bgColor, Color movingColor, int trailLength) {
    glow(bgColor);
    for (int i = currentTrailIndex; i < currentTrailIndex + trailLength; i++) {
      m_ledBuffer.setLED(i % m_ledBuffer.getLength(), movingColor);
    }
    currentTrailIndex++;
  }

  @Override
  public void periodic() {

    if(!DriverStation.isDSAttached()) {
      rainbow();
      m_led.setData(m_ledBuffer);
      return;
    } else {
      if(DriverStation.isDisabled()) {
        glow(255,0,0);
        m_led.setData(m_ledBuffer);
      } else if (DriverStation.isEnabled()) {
        if(Status.getIndexerStatus() == IndexerStatus.kShooting) {
          flash(0,255,0);
          m_led.setData(m_ledBuffer);
          return;
        }
        if(Status.getIntakeStatus() == IntakeStatus.kExtended) {
          flash(255,0,0);
          m_led.setData(m_ledBuffer);
          return;
        }
        flash(0,0,255);
        m_led.setData(m_ledBuffer);
      }
    }
  }
}