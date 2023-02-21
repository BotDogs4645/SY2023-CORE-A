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
import frc.robot.util.swervehelper.SwerveSettings;

public class PrecisionTeleop extends CommandBase {
  private Swerve s_Swerve;
  private JoystickAxisAIO translateY;
  private ProfiledPIDController pid;

  private double closestHeading = 0;
  private double[] headings = new double[] {0, Math.PI / 2, Math.PI, -Math.PI / 2};

  /** Creates a new PrecisionTeleop. */
  public PrecisionTeleop(
    Swerve s_Swerve,
    JoystickAxisAIO translateY
  ) {
    this.s_Swerve = s_Swerve;
    this.translateY = translateY;

    this.pid = new ProfiledPIDController(
      6.0,
      0.0,
      0.0,
      new TrapezoidProfile.Constraints(Math.toRadians(90), Math.toRadians(60))
    );

    pid.setTolerance(Math.toRadians(5), Math.toRadians(0.5));
    pid.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // get the rotation of the Robot's pose
    double robotRotation = s_Swerve.getPose().getRotation().getRadians();

    // see which is closer: 0, 90, 180, or 270 degrees
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

    Translation2d translation = new Translation2d(0, 0);

    if (pid.atGoal()) {
      if (closestHeading == headings[0] || closestHeading == headings[2]) {
        translation = new Translation2d(0, translateY.getValue())
          .times(SwerveSettings.driver.maxSpeed() * 0.3);
      } else {
        translation = new Translation2d(translateY.getValue(), 0)
          .times(SwerveSettings.driver.maxSpeed() * 0.3);
      }
    }

    s_Swerve.drive(
      s_Swerve.generateRequest(
        new Pose2d(translation, Rotation2d.fromRadians(newRotationInput)),
        true, 
        1.0
      )
    );
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
