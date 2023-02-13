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

public class ForcedHeadingTeleop extends CommandBase {
  private Swerve s_Swerve;
  private JoystickAxisAIO translateX;
  private JoystickAxisAIO translateY;
  private JoystickAxisAIO redKey;
  private double headingSupplier;

  private double baseSpeed = .6;
  private ProfiledPIDController rotationPID;
  private double newRotationInput;

  /** Creates a new ForcedHeadingTeleop. */
  public ForcedHeadingTeleop(
    Swerve s_Swerve,
    JoystickAxisAIO translateX,
    JoystickAxisAIO translateY,
    JoystickAxisAIO redKey,
    double headingSupplier
  ) {
    this.s_Swerve = s_Swerve;
    this.translateX = translateX;
    this.translateY = translateY;
    this.redKey = redKey;
    this.headingSupplier = headingSupplier;

    this.rotationPID = new ProfiledPIDController(
      6.0,
      0.0,
      0.0,
      new TrapezoidProfile.Constraints(Math.toRadians(220), Math.toRadians(160))
    );

    rotationPID.setTolerance(Math.toRadians(1), Math.toRadians(1));
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationPID.reset(
      new State(s_Swerve.getPose().getRotation().getRadians(), s_Swerve.speedVector.omegaRadiansPerSecond)
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translation = new Translation2d(
      -translateY.getValue(),
      -translateX.getValue()
    ).times(SwerveSettings.driver.maxSpeed() * baseSpeed + ((1.0 - baseSpeed) * (redKey.getValue() > .5 ? 1.0 : 0.0)));

    TrapezoidProfile.State finalState = new TrapezoidProfile.State(headingSupplier, 0);
    newRotationInput = rotationPID.calculate(s_Swerve.getPose().getRotation().getRadians(), finalState);

    s_Swerve.drive(
      s_Swerve.generateRequest(
        new Pose2d(translation, Rotation2d.fromRadians(newRotationInput)),
        true,
        1
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
