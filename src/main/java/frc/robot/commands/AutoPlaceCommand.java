// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pendulum;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Pendulum.PendulumControlRequest;
import frc.robot.subsystems.swerve.Swerve;


public class AutoPlaceCommand extends CommandBase {
  private Swerve swerve;
  private Pendulum pendulum;
  private Vision vision;

  private ProfiledPIDController translateXController;
  private ProfiledPIDController translateYController;
  private ProfiledPIDController rotateOmegaController;
  private TrapezoidProfile.State pendulumAngle;

  /** Creates a new ToPoseFromSnapshotGroup. */
  public AutoPlaceCommand(Pendulum pendulum, Swerve swerve, Vision vision) {
    translateXController = new ProfiledPIDController(
      2.5, 0, 0,
      new TrapezoidProfile.Constraints(2, 1)
    );
    
    translateYController = new ProfiledPIDController(
      2.5, 0, 0,
      new TrapezoidProfile.Constraints(2, 1)
    );

    rotateOmegaController = new ProfiledPIDController(
      0.5, 0, 0,
      new TrapezoidProfile.Constraints(Math.toRadians(20), Math.toRadians(10))
    );

    this.swerve = swerve;
    this.pendulum = pendulum;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, pendulum);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Grab most recent result
    PhotonPipelineResult result = vision.getCurrentCaptures();

    // This result is only captured if there is an apriltag in view. The one with best clarity will be the selected one.
    // Semantics technically deduces that the driver will only have the apriltag they're trying to target in their view, with how this will be set up.
    // TODO: handle null case
    PhotonTrackedTarget finalTarget = result.getBestTarget();
    
    // Use final target as the snapshot and determine the end effector pose
    Pose3d endEffector = 
      vision.getAprilTagPose(finalTarget.getFiducialId()) // get the AprilTag's pose
      .transformBy(vision.getSelectedAprilTagTransform().getTransform()) // move the pose left, not at all or right
      .transformBy(vision.getLocationToPlace().getTransform()) // move the pose to the placement position
    ;

    PendulumControlRequest goal = new PendulumControlRequest(endEffector);

    // Set the goal.
    translateXController.setGoal(new State(goal.getRobotPosition().getX(), 0));
    translateYController.setGoal(new State(goal.getRobotPosition().getY(), 0));
    rotateOmegaController.setGoal(new State(goal.getRobotPosition().getRotation().getRadians(), 0));
    pendulumAngle = goal.getPendulumRotation();

    // Grab the original pose. The reset() method of PIDControllers is always the **position**, not the velocity.
    // Velocity is something it calculates.
    Pose2d currentPose = swerve.getPose();
    translateXController.reset(currentPose.getX());
    translateYController.reset(currentPose.getY());
    rotateOmegaController.reset(currentPose.getRotation().getRadians());

    pendulum.zero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translation = new Translation2d(
      translateXController.calculate(swerve.getPose().getX()),
      translateYController.calculate(swerve.getPose().getY())
    );

    swerve.drive(translation, rotateOmegaController.calculate(swerve.getYaw().getRadians()), false);
    pendulum.move(pendulumAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return translateXController.atSetpoint() && 
        translateYController.atSetpoint() && 
        rotateOmegaController.atSetpoint();
  }
}
