package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Logger {

  public static void log(String subsystem, String name, String message) {
    SmartDashboard.putString(subsystem + "/" + name, message);
  }

  public static void log(String subsystem, String name, Double message) {
    SmartDashboard.putNumber(subsystem + "/" + name, message);
  }

  public static void log(String subsystem, String name, Boolean message) {
    SmartDashboard.putBoolean(subsystem + "/" + name, message);
  }

  public static void log(String subsystem, String name, Integer message) {
    SmartDashboard.putNumber(subsystem + "/" + name, message);
  }

  public static void log(String subsystem, String name, Object message) {
    SmartDashboard.putString(subsystem + "/" + name, message.toString());
  }

  public static void logWithNetworkTable(NetworkTable table, String name, String message) {
    table.getEntry(name).setString(message);
  }

  public static void logWithNetworkTable(NetworkTable table, String name, Double message) {
    table.getEntry(name).setDouble(message);
  }

  public static void logWithNetworkTable(NetworkTable table, String name, Boolean message) {
    table.getEntry(name).setBoolean(message);
  }

  public static void logWithNetworkTable(NetworkTable table, String name, int message) {
    table.getEntry(name).setNumber(message);
  }

  public static void logWithNetworkTable(NetworkTable table, String name, Object message) {
    table.getEntry(name).setString(message.toString());
  }
}
