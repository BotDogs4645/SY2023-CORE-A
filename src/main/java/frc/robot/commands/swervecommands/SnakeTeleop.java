// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervecommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.bdlib.driver.JoystickAxisAIO;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.swervehelper.SwerveSettings;

public class SnakeTeleop extends CommandBase {
  private Swerve s_Swerve;
  private JoystickAxisAIO translateX;
  private JoystickAxisAIO translateY;
  private JoystickAxisAIO redKey;
  private double baseSpeed = 0.6;

  private ProfiledPIDController pid;

  /** Creates a new SnakeTeleop. */
  public SnakeTeleop(
    Swerve s_Swerve,
    JoystickAxisAIO translateX,
    JoystickAxisAIO translateY,
    JoystickAxisAIO redKey
  ) {
    this.s_Swerve = s_Swerve;
    this.translateX = translateX;
    this.translateY = translateY;
    this.redKey = redKey;

    this.pid = new ProfiledPIDController(
      5.0,
      0.0,
      0.0,
      new TrapezoidProfile.Constraints(Math.toRadians(120), Math.toRadians(60))
    );

    pid.setTolerance(Math.toRadians(.05), Math.toRadians(1));
    pid.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset(
      new State(s_Swerve.getPose().getRotation().getRadians(), s_Swerve.speedVector.omegaRadiansPerSecond)
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // convert speed vector -> polar vector
    // rotate robot to face that vector
    // ezpz

    Translation2d translation = new Translation2d(
      -translateY.getValue(),
      -translateX.getValue()
    ).times(SwerveSettings.driver.maxSpeed() * baseSpeed + ((1.0 - baseSpeed) * (redKey.getValue() > .5 ? 1.0 : 0.0)));

    if (s_Swerve.getChassisSpeed() > 0.25) {
      snake(translation);
    } else {
      normalDrive(translation);
    }
  }

  private void snake(Translation2d translation) {
    double currentVelocityDirection = Rotation2d.fromRadians(Math.PI / 2).minus(Rotation2d.fromRadians(Math.atan(
      s_Swerve.speedVector.vxMetersPerSecond / s_Swerve.speedVector.vyMetersPerSecond
    ))).getRadians();

    TrapezoidProfile.State finalState = new TrapezoidProfile.State(currentVelocityDirection, 0);
    double newRotationInput = pid.calculate(s_Swerve.getPose().getRotation().getRadians(), finalState);

    s_Swerve.drive(
      s_Swerve.generateRequest(
        new Pose2d(translation, Rotation2d.fromRadians(newRotationInput)),
        true, 
        1.0
      )
    );
  }

  private void normalDrive(Translation2d translation) {
    pid.reset(
      new State(s_Swerve.getPose().getRotation().getRadians(), s_Swerve.speedVector.omegaRadiansPerSecond)
    );
    
    s_Swerve.drive(
      s_Swerve.generateRequest(
        new Pose2d(translation, Rotation2d.fromRadians(0)), // do not rotate.
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
