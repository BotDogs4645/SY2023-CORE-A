// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervecommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.bdlib.driver.JoystickAxisAIO;
import frc.robot.subsystems.swerve.Swerve;

public class PrecisionTeleop extends CommandBase {
  Swerve s_Swerve;
  JoystickAxisAIO translateX;
  ProfiledPIDController pid;

  double closestHeading = 0;
  double[] headings = new double[] {0, 90, 180, 270};

  /** Creates a new PrecisionTeleop. */
  public PrecisionTeleop(
    Swerve s_Swerve,
    JoystickAxisAIO translateX
  ) {
    this.s_Swerve = s_Swerve;
    this.translateX = translateX;

    this.pid = new ProfiledPIDController(
      0.2,
      0.0,
      0.0,
      new TrapezoidProfile.Constraints(.2, .1)
    );

    pid.setTolerance(Math.toRadians(10), Math.toRadians(1));
    pid.enableContinuousInput(0, Math.PI);

    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // get the rotation of the Robot's pose
    double robotRotation = s_Swerve.getPose().getRotation().getDegrees();

    // see which is closer, 180 degrees or 0
    double lowestDifference = Math.abs(robotRotation - headings[0]);
    closestHeading = 0;
    for (double dub: headings) {
      double newNum = Math.abs(robotRotation - dub);
      if (newNum < lowestDifference) {
        lowestDifference = newNum;
        closestHeading = dub;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TrapezoidProfile.State finalState = new TrapezoidProfile.State(closestHeading, 0);
    double newRotationInput = pid.calculate(s_Swerve.getPose().getRotation().getRadians(), finalState);

    if (pid.atGoal()) {
      // allow movement
      s_Swerve.drive(
        s_Swerve.generateRequest(
          new Pose2d(new Translation2d(), Rotation2d.fromRadians(newRotationInput)),
          true,
          .3
        )
      );

    } else {
      s_Swerve.drive(
        s_Swerve.generateRequest(
          new Pose2d(new Translation2d(), Rotation2d.fromRadians(newRotationInput)),
          true,
          1.0
        )
      );

    }
  }

  // public Translation2d getMovement() {

  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
