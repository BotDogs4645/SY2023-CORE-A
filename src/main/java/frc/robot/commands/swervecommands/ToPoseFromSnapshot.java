// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervecommands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.Swerve;


public class ToPoseFromSnapshot extends CommandBase {
  private Swerve swerve;
  private Vision vision;

  private Pose2d toMoveTo;

  private ProfiledPIDController translateXController;
  private ProfiledPIDController translateYController;
  private ProfiledPIDController rotateOmegaController;

  /** Creates a new ToPoseFromSnapshotGroup. */
  public ToPoseFromSnapshot(Swerve swerve, Vision vision) {
    translateXController = new ProfiledPIDController(2.5, 0, 0,
      new TrapezoidProfile.Constraints(2, 1)
    );
    
    translateYController = new ProfiledPIDController(
      2.5, 0, 0,
      new TrapezoidProfile.Constraints(2, 1)
    );

    rotateOmegaController = new ProfiledPIDController(0.5, 0,0,
      new TrapezoidProfile.Constraints(Math.toRadians(20), Math.toRadians(10))
    );

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Grab most recent result
    PhotonPipelineResult result = vision.getCurrentCaptures();

    // This result is only captured if there is an apriltag in view. The one with best clarity will be the selected one.
    // Semantics technically deduces that the driver will only have the apriltag they're trying to target in their view, with how this will be set up.
    PhotonTrackedTarget finalTarget = result.getBestTarget();
    
    // Use final target as the snapshot and determine the Pose to move to
    toMoveTo = 
      vision.getAprilTagPose(finalTarget.getFiducialId()).toPose2d()
      .transformBy(vision.getSelectedAprilTagTransform().getTransform());

    // Set the goal.
    translateXController.setGoal(new State(toMoveTo.getX(), 0));
    translateYController.setGoal(new State(toMoveTo.getY(), 0));
    rotateOmegaController.setGoal(new State(toMoveTo.getRotation().getRadians(), 0));


    // Grab the original pose. The reset() method of PIDControllers is always the **position**, not the velocity.
    // Velocity is what it calculates.
    Pose2d currentPose = swerve.getPose();
    translateXController.reset(currentPose.getX());
    translateYController.reset(currentPose.getY());
    rotateOmegaController.reset(currentPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translation = new Translation2d(
      translateXController.calculate(swerve.getPose().getX()),
      translateYController.calculate(swerve.getPose().getY())
    );

    swerve.drive(translation, rotateOmegaController.calculate(swerve.getYaw().getRadians()), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return translateXController.atSetpoint() && translateYController.atSetpoint() && rotateOmegaController.atSetpoint();
  }
}
