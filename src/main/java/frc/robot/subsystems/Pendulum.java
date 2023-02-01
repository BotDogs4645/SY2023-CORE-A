// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bdlib.custom_talon.TalonFXW;
import frc.robot.Constants.PendulumConstants;

public class Pendulum extends SubsystemBase {
  TalonFXW left, center, right;
  MotorControllerGroup falconGearbox;

  /* State space components */
  private LinearSystem<N2, N1, N1> pendulumPlant;
  private KalmanFilter<N2, N1, N1> observer;
  private LinearQuadraticRegulator<N2, N1, N1> LQR;

  private LinearSystemLoop<N2, N1, N1> controlLoop;


  public Pendulum() {
    /* Motor declarations - keep it CTRE && on RIO can bus ("") */
    this.left = new TalonFXW(PendulumConstants.leftId, PendulumConstants.pendulumFalconsConfig);
    this.center = new TalonFXW(PendulumConstants.centerId, PendulumConstants.pendulumFalconsConfig);
    this.right = new TalonFXW(PendulumConstants.rightId, PendulumConstants.pendulumFalconsConfig);
    
    this.falconGearbox = new MotorControllerGroup(left, center, right);

    /* State space stuff */
    // Pendulum arm model
    this.pendulumPlant = LinearSystemId.createSingleJointedArmSystem(
      DCMotor.getFalcon500(PendulumConstants.numOfMotors),
      PendulumConstants.momentOfInertia,
      PendulumConstants.gearing
    );

    // Observer. Rejects noise as much as possible using a KalmanFilter
    this.observer = new KalmanFilter<>(
      Nat.N2(),
      Nat.N1(),
      pendulumPlant,
      VecBuilder.fill(0.015, 0.17), // model accuracy std dev.
      VecBuilder.fill(0.01), // encoder accuracy std dev. low because we trust it
      0.020
    );
    
    this.LQR = new LinearQuadraticRegulator<>(
      pendulumPlant,
      VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0)), // qelms.
      // Position and velocity error tolerances, in radians and radians per second. Decrease
      // this to more heavily penalize state excursion, or make the controller behave more
      // aggressively.
      VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
      // heavily penalize control effort, or make the controller less aggressive.
      0.020
    );

    this.controlLoop = new LinearSystemLoop<>(pendulumPlant, LQR, observer, 12.0, 0.020);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
