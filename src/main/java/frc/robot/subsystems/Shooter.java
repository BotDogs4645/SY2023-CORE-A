// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonSRX shooter;
  
  /** Creates a new Shooter. */
  public Shooter() {
    this.shooter = new TalonSRX(Constants.shooterMotorId);
  }

  public void setSpeed(double speed) {
    shooter.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {}
}
