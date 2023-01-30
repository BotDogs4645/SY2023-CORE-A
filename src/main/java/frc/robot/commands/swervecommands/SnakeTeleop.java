// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.bdlib.driver.JoystickAxisAIO;
import frc.robot.subsystems.swerve.Swerve;

public class SnakeTeleop extends CommandBase {
  private Swerve s_Swerve;
  private JoystickAxisAIO translateX;
  private JoystickAxisAIO translateY;
  private JoystickAxisAIO redKey;

  /** Creates a new SnakeTeleop. */
  public SnakeTeleop(
    Swerve s_Swerve,
    JoystickAxisAIO translateX,
    JoystickAxisAIO translateY,
    JoystickAxisAIO redKey
  ) {
    this.s_Swerve = s_Swerve;
    this.translateX = translateX;
    this.translateY = translateY;
    this.redKey = redKey;

    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
