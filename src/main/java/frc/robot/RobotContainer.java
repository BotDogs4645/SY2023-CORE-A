// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.bdlib.driver.ControllerAIO;
import frc.bdlib.driver.JoystickAxisAIO;
import frc.bdlib.misc.BDConstants.JoystickConstants.JoystickAxisID;
import frc.bdlib.misc.BDConstants.JoystickConstants.JoystickButtonID;
import frc.robot.Constants.PendulumConstants.PendulumCommand;
import frc.robot.commands.autos.ExampleAuto1;
import frc.robot.commands.autos.ExampleCommand;
import frc.robot.commands.pendulumcommands.AutoPlaceCommand;
import frc.robot.commands.pendulumcommands.SetVisionSettings;
import frc.robot.commands.swervecommands.TeleopSwerve;
import frc.robot.commands.swervecommands.TeleopSwerveAroundPoint;
import frc.robot.subsystems.*;
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
  private final Pendulum pendulum = new Pendulum();
  private final Vision vision = new Vision();

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    Shuffleboard.getTab("auto").add(autoChooser);
    // JoyRumbler rumbler = new JoyRumbler(driver, driver.getToggleBooleanSupplier(JoystickButtonID.kX, 2));
    // rumbler.addRumbleScenario(() -> true, RumblerType.LEFT_SHAKER);

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
    /* Axis Controllers */
    JoystickAxisAIO leftXAxis = driver.getAxis(JoystickAxisID.kLeftX, SwerveSettings.driver.leftX());
    JoystickAxisAIO leftYAxis = driver.getAxis(JoystickAxisID.kLeftY, SwerveSettings.driver.leftY());
    JoystickAxisAIO rightXAxis = driver.getAxis(JoystickAxisID.kRightX, SwerveSettings.driver.rightX());

    /* Driver Buttons */
    driver.getJoystickButton(JoystickButtonID.kX)
      .onTrue(
        new ConditionalCommand(
            new AutoPlaceCommand(pendulum, swerve, vision),
            new InstantCommand(),
            vision::hasTargets
          )
      );
    
    driver.getJoystickButton(JoystickButtonID.kA)
      .onTrue(new InstantCommand(() -> {
        swerve.zeroGyro();
      }));

    // driver.getJoystickButton(JoystickButtonID.kY)
    // .toggleOnTrue(new OrientationFlipCommand(
    // swerve, leftXAxis, leftYAxis
    // )
    // );

    driver.getJoystickButton(JoystickButtonID.kY)
      .toggleOnTrue(new TeleopSwerveAroundPoint(swerve, leftXAxis, leftYAxis, rightXAxis)
    );

    // driver.getJoystickButton(JoystickButtonID.kY)
    // .toggleOnTrue(new RotateAroundAbsolutePoint(
    // swerve, leftXAxis, leftYAxis, rightXAxis,
    // () -> {return new Translation2d();}
    // ));

    /* Manipulator Buttons */
    // TODO: put in manipulator setup in other branch
    JoystickAxisAIO trigger = manipulator.getAxis(JoystickAxisID.kRightTrigger, JoystickAxisAIO.LINEAR);
    new Trigger(trigger.axisHigherThan(.5))
      .onTrue(new SetVisionSettings(manipulator, vision));

    // Vision bindings

    /* Default Commands */
    swerve.setDefaultCommand(
      new TeleopSwerve(
        swerve, leftXAxis, leftYAxis, rightXAxis, false
      )
    );

    pendulum.setDefaultCommand(
      new InstantCommand(() -> {
        // default position is arm facing down @ velocity 0, waiting for position commands
        pendulum.move(new TrapezoidProfile.State(PendulumCommand.Idle.get(), 0.0));
      }
    ));

    new RunCommand(() -> {
      Optional<Pose2d> possible_pose = vision.getRobotPoseContributor();
      if (possible_pose.isPresent()) {
        swerve.provideVisionInformation(possible_pose.get());
      }
    })
    .ignoringDisable(true)
    .schedule();
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
