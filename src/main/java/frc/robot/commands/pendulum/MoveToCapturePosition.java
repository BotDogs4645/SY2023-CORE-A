// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pendulum;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.bdlib.driver.JoystickAxisAIO;
import frc.robot.Constants.PendulumConstants.PendulumCommand;
import frc.robot.subsystems.Pendulum;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.swervehelper.SwerveSettings;

public class MoveToCapturePosition extends CommandBase {
  private Swerve swerve;
  private Pendulum pendulum;
  private JoystickAxisAIO translateX;
  private JoystickAxisAIO translateY;

  private ProfiledPIDController rotationController;
  private TrapezoidProfile.State rotationalState = new TrapezoidProfile.State(0, 0);

  /** Creates a new MoveToCapturePosition. */
  public MoveToCapturePosition(
      Swerve swerve,
      Pendulum pendulum,
      JoystickAxisAIO translateX,
      JoystickAxisAIO translateY
    ) {
    this.swerve = swerve;
    this.pendulum = pendulum;
    this.translateX = translateX;
    this.translateY = translateY;
    
    this.rotationController = new ProfiledPIDController(
      5.0, 
      0,
      0,
      new TrapezoidProfile.Constraints(Math.toRadians(90), Math.toRadians(60))
    );

    rotationController.setTolerance(Math.toRadians(1), Math.toRadians(10));
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve, pendulum);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.reset(swerve.getYaw().getRadians(), 0);
    rotationController.setGoal(rotationalState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double newRotationInput = rotationController.calculate(swerve.getPose().getRotation().getRadians(), rotationalState);

    Translation2d translation = new Translation2d(-translateX.getValue(), -translateY.getValue())
    .times(SwerveSettings.driver.maxSpeed() * 0.5);

    swerve.drive(
      swerve.generateRequest(
        new Pose2d(translation, Rotation2d.fromRadians(newRotationInput)),
        true, 
        1.0
      )
    );
    pendulum.move(new TrapezoidProfile.State(PendulumCommand.RetrieveAngle.get(), 0));
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