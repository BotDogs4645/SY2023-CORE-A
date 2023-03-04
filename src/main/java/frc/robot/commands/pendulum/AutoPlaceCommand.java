// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pendulum;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.bdlib.driver.ControllerAIO;
import frc.bdlib.driver.ToggleBooleanSupplier;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Pendulum;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Pendulum.PendulumControlRequest;
import frc.robot.subsystems.Vision.CameraType;
import frc.robot.subsystems.swerve.Swerve;


public class AutoPlaceCommand extends CommandBase {
  private Swerve swerve;
  private Pendulum pendulum;
  private Vision vision;
  private Claw claw;
  private ControllerAIO aio;
  private ToggleBooleanSupplier rumbleMuter;

  private Pose3d currentEndEffector;
  private PendulumControlRequest currentRequest;

  private PIDController translateXController;
  private PIDController translateYController;
  private ProfiledPIDController rotateOmegaController;

  private double maxMetersChangePerSecond = 0.25;
  private boolean canDrop = false;

  /** Creates a new ToPoseFromSnapshotGroup. */
  public AutoPlaceCommand(
      Pendulum pendulum, 
      Swerve swerve, 
      Vision vision, 
      Claw claw, 
      ControllerAIO aio, 
      ToggleBooleanSupplier rumbleMuter
    ) {
    translateXController = new PIDController(
      2.5, 0, 0
    );
    
    translateYController = new PIDController(
      2.5, 0, 0
    );

    rotateOmegaController = new ProfiledPIDController(
      5.0, 0, 0,
      new TrapezoidProfile.Constraints(Math.toRadians(90), Math.toRadians(60))
    );

    this.swerve = swerve;
    this.pendulum = pendulum;
    this.claw = claw;
    this.rumbleMuter = rumbleMuter;

    translateXController.setTolerance(0.5, 0.05);
    translateYController.setTolerance(0.5, 0.05);

    rotateOmegaController.setTolerance(Math.toRadians(1), Math.toRadians(10));
    rotateOmegaController.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, pendulum, claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    canDrop = false;

    // Grab most recent result
    PhotonPipelineResult result = vision.getCurrentCaptures();

    // This result is only captured if there is an apriltag in view. The one with best clarity will be the selected one.
    // Semantics technically deduces that the driver will only have the apriltag they're trying to target in their view, with how this will be set up.
    // TODO: handle null case
    PhotonTrackedTarget finalTarget = result.getBestTarget();
    
    // Use final target as the snapshot and determine the end effector pose
    currentEndEffector = 
      vision.getAprilTagPose(finalTarget.getFiducialId()) // get the AprilTag's pose
      .transformBy(vision.getSelectedAprilTagTransform().getTransform())
      .transformBy(vision.getLocationToPlace().getTransform());

    currentRequest = new PendulumControlRequest(currentEndEffector);

    // Set the goal.
    translateXController.setSetpoint(currentRequest.getRobotPosition().getX());
    translateYController.setSetpoint(currentRequest.getRobotPosition().getY());
    rotateOmegaController.setGoal(new State(currentRequest.getRobotPosition().getRotation().getRadians(), 0));
    pendulum.setGoal(currentRequest.getPendulumRotation());

    // Grab the original pose. The reset() method of PIDControllers is always the **position**, not the velocity.
    // Velocity is something it calculates.
    Pose2d currentPose = swerve.getPose();
    translateXController.reset();
    translateYController.reset();
    rotateOmegaController.reset(currentPose.getRotation().getRadians());

    vision.setDriverCamera(CameraType.Arm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (aio.getPOV() != -1) {
      // yeah they want to change the end effector.. bruh..
      double rads = Math.toRadians(aio.getPOV());
      double changeInX = -(Math.cos(rads) * 0.020) * maxMetersChangePerSecond; // ask dave if u wanna know why this is negative
      double changeInY = (Math.sin(rads) * 0.020) * maxMetersChangePerSecond;
      currentEndEffector = currentEndEffector.plus(new Transform3d(new Translation3d(changeInX, changeInY, 0), new Rotation3d()));
      currentRequest = new PendulumControlRequest(currentEndEffector);
    }

    Translation2d translation = new Translation2d(
      translateXController.calculate(swerve.getPose().getX(), currentRequest.getRobotPosition().getX()),
      translateYController.calculate(swerve.getPose().getY(), currentRequest.getRobotPosition().getY())
    );

    Rotation2d rotation = Rotation2d.fromRadians(
      rotateOmegaController.calculate(
        swerve.getYaw().getRadians(),
        new State(currentRequest.getRobotPosition().getRotation().getRadians(),
        0)
      )
    );

    swerve.drive(swerve.generateRequest(new Pose2d(translation, rotation), false, 1.0));
    pendulum.setGoal(currentRequest.getPendulumRotation());
    claw.setAmperage(5.5);

    if (
      !rumbleMuter.getValue() &&
      pendulum.atSetpoint() &&
      translateXController.atSetpoint() && 
      translateYController.atSetpoint() &&
      rotateOmegaController.atGoal()
    ) {
      // we are at the setpoint!
      aio.setRumble(RumbleType.kBothRumble, 1.0);
      canDrop = true;
    } else {
      aio.setRumble(RumbleType.kBothRumble, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (canDrop) {
      new RunCommand(() -> claw.setAmperage(-5.5), claw).until(() -> !claw.switchPressed());
    }
    vision.setDriverCamera(CameraType.Robot);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}