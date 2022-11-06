// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.Limelight;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurretAutoAim extends PIDCommand {
  /** Creates a new TurretAutoAim. */
  public TurretAutoAim(TurretSubsystem turret) {
    super(new PIDController(.1, 0, 0), Limelight::getTx, () -> 0,
        output -> turret.runTurret(output));
    getController().enableContinuousInput(-70, 70);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
