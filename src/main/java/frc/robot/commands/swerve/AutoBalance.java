// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Pendulum;
import frc.robot.subsystems.swerve.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends ProfiledPIDCommand {
  Pendulum pendulum;
  Swerve swerve;

  /** Creates a new AutoBalance. */
  public AutoBalance(Swerve swerve, Pendulum pendulum) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.25,
            0,
            100,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0)),
        // This should return the measurement
        () -> Math.toRadians(swerve.getGyroRotationComponents()[1]),
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(0, 0),
        // This uses the output
        (output, setpoint) -> {
          swerve.driveNoFOC(new Pose2d(new Translation2d(output, 0), new Rotation2d()), 1.0);
        });
    this.pendulum = pendulum;
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(swerve, pendulum);
    super.getController().setTolerance(Math.toRadians(5), Math.toRadians(0.01));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
