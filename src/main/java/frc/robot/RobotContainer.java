// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.bdlib.driver.ControllerAIO;
import frc.bdlib.driver.JoystickAxisAIO;
import frc.bdlib.driver.ToggleBooleanSupplier;
import frc.bdlib.misc.BDConstants.JoystickConstants.JoystickAxisID;
import frc.bdlib.misc.BDConstants.JoystickConstants.JoystickButtonID;
import frc.robot.Constants.PendulumConstants.PendulumCommand;
import frc.robot.commands.autos.ExampleAuto1;
import frc.robot.commands.autos.ExampleCommand;
import frc.robot.commands.pendulum.AutoPlaceCommand;
import frc.robot.commands.pendulum.MoveToCapturePosition;
import frc.robot.commands.pendulum.SetVisionSettings;
import frc.robot.commands.swerve.NormalTeleop;
import frc.robot.commands.swerve.PrecisionTeleop;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Pendulum;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.swervehelper.SwerveSettings;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final ControllerAIO driver = new ControllerAIO(0);
  private final ControllerAIO manipulator = new ControllerAIO(1);

  /* Subsystems */
  private final Swerve swerve = new Swerve();
  private final Vision vision = new Vision();
  private final Pendulum pendulum = new Pendulum();
  private final Claw claw = new Claw();
  private final Shooter shootie = new Shooter();

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    Shuffleboard.getTab("auto").add(autoChooser);

    // Configure the button bindings
    configureButtonBindings();
    // Configure autonomous mode
    configureAutonomous();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Manipulator Buttons */
    configureDriverController();
    configureManipulatorController();

    // Vision bindings
    new RunCommand(() -> {
      List<Optional<EstimatedRobotPose>> possiblePoses = vision.getRobotPoseContributor();
      swerve.provideVisionInformation(
        possiblePoses.stream().filter((item) -> item.isPresent()).map((item) -> item.get()).toList()
      );
    })
      .ignoringDisable(true)
      .schedule();
  }

  public void configureDriverController() {
    /* Axis Controllers */
    JoystickAxisAIO leftXAxis = driver.getAxis(JoystickAxisID.kLeftX, SwerveSettings.driver.leftX());
    JoystickAxisAIO leftYAxis = driver.getAxis(JoystickAxisID.kLeftY, SwerveSettings.driver.leftY());
    JoystickAxisAIO rightXAxis = driver.getAxis(JoystickAxisID.kRightX, SwerveSettings.driver.rightX());
    JoystickAxisAIO rightTrigger = driver.getAxis(JoystickAxisID.kRightTrigger, JoystickAxisAIO.LINEAR);
    JoystickAxisAIO autoPlaceTrigger = driver.getAxis(JoystickAxisID.kLeftTrigger, JoystickAxisAIO.LINEAR);
    ToggleBooleanSupplier muteRumblerButton = driver.getToggleBooleanSupplier(JoystickButtonID.kY, 0.5);

    new Trigger(autoPlaceTrigger.axisHigherThan(.5))
      .onTrue(
        new ConditionalCommand(
            new AutoPlaceCommand(pendulum, swerve, vision, claw, driver, muteRumblerButton),
            new InstantCommand(),
            () -> vision.hasTargets() && claw.switchPressed()
          )
      );
    
    driver.getJoystickButton(JoystickButtonID.kA)
      .onTrue(new InstantCommand(swerve::zeroGyro)
    );

    /* Driver Mode Semantics */
    // Set the normal teleop command.
    swerve.setDefaultCommand(new NormalTeleop(swerve, leftXAxis, leftYAxis, rightXAxis,
      driver.getJoystickButton(JoystickButtonID.kRightBumper), rightTrigger)
    );

    // Sets the normal teleop command
    pendulum.setDefaultCommand(
      new RunCommand(() -> {
        // default position is arm facing down @ velocity 0, waiting for position commands
        pendulum.move(new TrapezoidProfile.State(PendulumCommand.Straight.get() + Math.toRadians(7), 0.0));
      }, pendulum
    ));

    Commands.sequence(
      Commands.run(() -> shootie.setSpeed(1), shootie),
      Commands.waitSeconds(3),
      Commands.runOnce(() -> shootie.setSpeed(0), shootie)
    ).schedule();

    // Other types of modes
    // Precision mode
    driver.getJoystickButton(JoystickButtonID.kX)
      .whileTrue(
        new PrecisionTeleop(swerve, leftXAxis)
      );

    driver.getJoystickButton(JoystickButtonID.kB)
      .onTrue(
        new MoveToCapturePosition(swerve, pendulum, leftXAxis, leftYAxis)
      );

    // Recenter gyro mode
    driver.getJoystickButton(JoystickButtonID.kBack)
      .toggleOnTrue(new InstantCommand(() -> {
        swerve.zeroGyro();
      })
    );

    driver.getJoystickButton(JoystickButtonID.kStart)
      .onTrue(new InstantCommand(() -> {
        pendulum.zeroPendulum();
      })
    );

    var closeButton = driver.getJoystickButton(JoystickButtonID.kLeftBumper);
    var override = driver.getJoystickButton(JoystickButtonID.kA);
    claw.setDefaultCommand(
            new RunCommand(() -> {
              // If the switch is pressed or we're currently closing, we should close
              boolean normalClose = claw.switchPressed() || closeButton.getAsBoolean();
              if (normalClose && !override.getAsBoolean()) { // Always open if the open override button is pressed
                claw.setAmperage(-5.5);
              } else {
                claw.setAmperage(9);
              }
            }, claw));
  }

  public void configureManipulatorController() {
    // vision settings
    JoystickAxisAIO settingsChangeTrigger = manipulator.getAxis(JoystickAxisID.kRightTrigger, JoystickAxisAIO.LINEAR);
    new Trigger(settingsChangeTrigger.axisHigherThan(.5))
      .onTrue(new SetVisionSettings(manipulator, vision));
  }

  private void configureAutonomous() {
    // when swerve reaches the event labeled "fire_ball", ExampleCommand will run.
    // note: swerve's path will not resume until ExampleCommand finishes unless,
    // it is set as a command that can run in parallel :)
    swerve.addEvent("fire_ball", new ExampleCommand());
    autoChooser.addOption("full auto 1", new ExampleAuto1(swerve));
    autoChooser.addOption("path important", swerve.getFullAutoPath(SwerveSettings.PathList.Path2));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getChooser() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}