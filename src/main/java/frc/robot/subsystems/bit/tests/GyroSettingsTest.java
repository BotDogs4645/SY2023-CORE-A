// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.bit.tests;

import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.bit.BITManager;
import frc.robot.subsystems.swerve.Swerve;

public class GyroSettingsTest extends CommandBase {
  private Swerve swerve;
  private BITManager manager;
  private double originalRotation;
  private double endRotation;

  private boolean finished = false;

  /** Creates a new GyroSettingsTest. */
  public GyroSettingsTest(BITManager manager, Swerve swerve) {
    this.manager = manager;
    this.swerve = swerve;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    originalRotation = swerve.getYaw().getRadians();
    manager.getIndicatorWidget().getEntry().accept(NetworkTableValue.makeBoolean(true));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(swerve.getYaw().getRadians() - originalRotation) > (Math.PI / 4)) {
      endRotation = swerve.getYaw().getRadians() - originalRotation;
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manager.getIndicatorWidget().getEntry().accept(NetworkTableValue.makeBoolean(true));
    if (endRotation > 0) {
      System.out.println("pass");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
