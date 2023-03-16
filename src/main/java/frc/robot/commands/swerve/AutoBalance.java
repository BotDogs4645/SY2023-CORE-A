// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Pendulum;
import frc.robot.subsystems.swerve.Swerve;

public class AutoBalance extends CommandBase {
  private Swerve swerve;
  private Pendulum pendulum;
  private Claw claw;
  private double angleDegrees;

  private double velocityThreshold = 0.5;
  private double inchesSpeed = 1.0;
  private double positionThreshold = 4;

  /** Creates a new AutoBalance. */
  public AutoBalance(Swerve swerve, Pendulum pendulum, Claw claw) {
    this.swerve = swerve;
    this.pendulum = pendulum;
    this.claw = claw;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, pendulum, claw);
    Shuffleboard.getTab("Auto Balance")
      .addNumber("pitch", () -> swerve.getPitch().getDegrees());
    Shuffleboard.getTab("Auto Balance")
      .addNumber("roll", () -> swerve.getRoll().getDegrees());

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleDegrees = Double.POSITIVE_INFINITY;
    pendulum.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pendulum.setGoal(new TrapezoidProfile.State(Math.toRadians(-85), 0));
    claw.setAmperage(-5.5);

    angleDegrees = swerve.getYaw().getCos() * swerve.getPitch().getDegrees() +
        swerve.getYaw().getSin() * swerve.getRoll().getDegrees();

    double angleVelocityDegreesPerSec = swerve.getYaw().getCos() * swerve.getPitchVelocity().getDegrees() +
        swerve.getYaw().getSin() * swerve.getRollVelocity().getDegrees();

    boolean shouldStop = (angleDegrees < 0.0 && angleVelocityDegreesPerSec > velocityThreshold)
        || (angleDegrees > 0.0 && angleVelocityDegreesPerSec < -velocityThreshold);

    if (!shouldStop) {
      swerve.drive(
        swerve.generateRequest(
          new Pose2d(new Translation2d(inchesSpeed * (angleDegrees > 0.0 ? 1.0 : 1.0), 0), Rotation2d.fromRadians(0)),
          true, 
          1.0
        )
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(angleDegrees) < positionThreshold;
  }
}