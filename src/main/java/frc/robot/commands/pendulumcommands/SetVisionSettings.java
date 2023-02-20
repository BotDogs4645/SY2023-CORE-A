// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pendulumcommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.bdlib.driver.ControllerAIO;
import frc.robot.Constants.AutoPositionConstants;
import frc.robot.Constants.AutoPositionConstants.AprilTagTransformDirection;
import frc.robot.Constants.AutoPositionConstants.GamePiecePlacementLevel;
import frc.robot.subsystems.Vision;

// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetVisionSettings extends InstantCommand {
  ControllerAIO manipulator;
  Vision vision;

  public SetVisionSettings(ControllerAIO manipulator, Vision vision) {
    this.manipulator = manipulator;
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set current settings fr fr
    // Gets a stream of the placement buttons we use
    AutoPositionConstants.placementButtons.keySet().stream()
        // Filters that stream for any buttons that are currently pressed
        .filter(button -> manipulator.getJoystickButton(button).getAsBoolean())
        // Gets the first option, could be anything really.
        .findFirst()
        // If there are no buttons pressed, deal with that here.
        .ifPresentOrElse(
          (present) -> {
            // Button was found, so we send over that button and we send over the POV
            // settings.
            vision.setPlacementSettings(
              AutoPositionConstants.placementButtons.get(present),
              AutoPositionConstants.getPoseAlignmentFromNumber(manipulator.getPOV())
            );
          },
            // If there were no buttons pressed, deal with that here
          () -> {
            // Set it to middle row (cube) & middle row.
            vision.setPlacementSettings(
              GamePiecePlacementLevel.MIDDLE,
              AprilTagTransformDirection.CENTER
            );
          }
      );
  }
}
