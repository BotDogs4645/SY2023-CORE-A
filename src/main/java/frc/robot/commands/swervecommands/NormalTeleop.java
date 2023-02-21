// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervecommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.bdlib.driver.JoystickAxisAIO;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.swervehelper.SwerveSettings;

public class NormalTeleop extends CommandBase {
  private Swerve s_Swerve;
  private JoystickAxisAIO translateX;
  private JoystickAxisAIO translateY;
  private JoystickAxisAIO rotationTheta;
  private JoystickButton eBrake;
  private JoystickAxisAIO redKey;

  Rotation2d currentVelocityDirection = new Rotation2d();

  private double baseSpeed = .6;

  /** Creates a new TeleopController. */
  public NormalTeleop(
      Swerve s_Swerve,
      JoystickAxisAIO translateX,
      JoystickAxisAIO translateY,
      JoystickAxisAIO rotationTheta,
      JoystickButton eBrake,
      JoystickAxisAIO redKey
    ) {
    this.s_Swerve = s_Swerve;
    this.translateX = translateX;
    this.translateY = translateY;
    this.rotationTheta = rotationTheta;
    this.eBrake = eBrake;
    this.redKey = redKey;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (eBrake.getAsBoolean()) {
      move(); 
    } else {
      brake();
    }
  }

  public void move() {
    Translation2d translation = new Translation2d(
      -translateY.getValue(),
      -translateX.getValue()
    ).times(SwerveSettings.driver.maxSpeed());

    Rotation2d rotation = Rotation2d.fromRadians(
      -rotationTheta.getValue() * SwerveSettings.driver.maxRotationSpeed()
    );

    s_Swerve.drive(
      s_Swerve.generateRequest(
        new Pose2d(translation, rotation),
        true,
        baseSpeed + ((1.0 - baseSpeed) * (redKey.getValue() > .5 ? 1.0 : 0.0))
      )
    );
  }

  public void brake() {
    if (s_Swerve.getChassisSpeed() > .5) {
      currentVelocityDirection = Rotation2d.fromRadians(Math.tan(
        s_Swerve.speedVector.vyMetersPerSecond / s_Swerve.speedVector.vxMetersPerSecond
      )).plus(Rotation2d.fromRadians(Math.PI / 2));
    }
    s_Swerve.nTable.putValue("velo", NetworkTableValue.makeDouble(currentVelocityDirection.getDegrees()));
    
    s_Swerve.drive(
      s_Swerve.generateRequest(
        new Pose2d(new Translation2d(.1, currentVelocityDirection), new Rotation2d()),
        true,
        1.0
      )
    );
  }
}
