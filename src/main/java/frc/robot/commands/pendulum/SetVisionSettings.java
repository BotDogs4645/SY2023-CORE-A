// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pendulum;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.bdlib.driver.ControllerAIO;
import frc.bdlib.misc.BDConstants.JoystickConstants.JoystickButtonID;
import frc.robot.Constants.AutoPositionConstants;
import frc.robot.Constants.AutoPositionConstants.AprilTagTransformDirection;
import frc.robot.Constants.AutoPositionConstants.GamePiecePlacementLevel;
import frc.robot.subsystems.Vision;

// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetVisionSettings extends InstantCommand {
  private ControllerAIO manipulator;
  private Vision vision;
  
  private GamePiecePlacementLevel currentPlacementLevel;
  private AprilTagTransformDirection currentTransformDirection;

  public SetVisionSettings(ControllerAIO manipulator, Vision vision) {
    this.manipulator = manipulator;
    this.vision = vision;

    this.currentPlacementLevel = GamePiecePlacementLevel.MIDDLE;
    this.currentTransformDirection = AprilTagTransformDirection.CENTER;
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    // Set current settings fr fr
    Optional<JoystickButtonID> check = AutoPositionConstants.placementButtons.keySet().stream()
      .filter(button -> manipulator.getJoystickButton(button).getAsBoolean())
      .findFirst();

    if (check.isPresent() && AutoPositionConstants.placementButtons.get(check.get()) != currentPlacementLevel) {
      currentPlacementLevel = AutoPositionConstants.placementButtons.get(check.get());
    }

    if (manipulator.getPOV() != -1 && AutoPositionConstants.getPoseAlignmentFromNumber(manipulator.getPOV()) != currentTransformDirection) {
      currentTransformDirection = AutoPositionConstants.getPoseAlignmentFromNumber(manipulator.getPOV());
    }

    vision.setPlacementSettings(currentPlacementLevel, currentTransformDirection);
  }
}
