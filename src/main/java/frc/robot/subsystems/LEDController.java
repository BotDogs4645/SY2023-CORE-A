// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDController extends SubsystemBase {
  public double YELLOW_INPUT = 0.69;
  public double VIOLET_INPUT = 0.91;
  public double BLACK_INPUT = 0.99;

  private Spark driver;
  /** Creates a new LEDController. */
  public LEDController() {
    this.driver = new Spark(8);
    setBlack();
  }

  public void setCone() {
    driver.set(YELLOW_INPUT);
  }

  public void setCube() {
    driver.set(VIOLET_INPUT);
  }

  public void setBlack() {
    driver.set(BLACK_INPUT);
  }

  public Command burstCone() {
    return new SequentialCommandGroup(
      new RunCommand(this::setCone, this),
      new WaitCommand(2),
      new RunCommand(this::setBlack, this)
    );
  }

  public Command burstCube() {
    return new SequentialCommandGroup(
      new RunCommand(this::setCube, this),
      new WaitCommand(2),
      new RunCommand(this::setBlack, this)
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
