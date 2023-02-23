// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pendulumcommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pendulum;
import frc.robot.subsystems.swerve.Swerve;

public class MoveToCapturePosition extends CommandBase {
  private Swerve swerve;
  private Pendulum pendulum;

  private ProfiledPIDController rotationController;

  /** Creates a new MoveToCapturePosition. */
  public MoveToCapturePosition(Swerve swerve, Pendulum pendulum) {
    this.swerve = swerve;
    this.pendulum = pendulum;
    
    this.rotationController = new ProfiledPIDController(
      5.0, 
      0,
      0,
      new TrapezoidProfile.Constraints(Math.toRadians(90), Math.toRadians(60))
    );

    addRequirements(swerve, pendulum);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.reset(swerve.getYaw().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
