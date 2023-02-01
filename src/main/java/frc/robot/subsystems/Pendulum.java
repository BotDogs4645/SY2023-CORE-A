// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bdlib.custom_talon.TalonFXW;
import frc.robot.Constants.PendulumConstants;
import frc.robot.util.swervehelper.Conversions;

public class Pendulum extends SubsystemBase {
  private TalonFXW[] motorGearbox;
  private CANCoder absoluteEncoder;

  /* State space components */
  private LinearSystem<N2, N1, N1> pendulumPlant;
  private KalmanFilter<N2, N1, N1> observer;
  private LinearQuadraticRegulator<N2, N1, N1> LQR;

  private LinearSystemLoop<N2, N1, N1> controlLoop;


  public Pendulum() {
    /* Motor declarations - keep it CTRE && on RIO can bus ("") */
    motorGearbox = new TalonFXW[] {
      new TalonFXW(PendulumConstants.leftId, PendulumConstants.pendulumFalconsConfig),
      new TalonFXW(PendulumConstants.centerId, PendulumConstants.pendulumFalconsConfig),
      new TalonFXW(PendulumConstants.rightId, PendulumConstants.pendulumFalconsConfig)
    };

    absoluteEncoder = new CANCoder(PendulumConstants.cancoderId);
    // we go -180-180 to represent the negative as below the horizon
    absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    zero();

    /* State space stuff */
    // Pendulum arm model
    this.pendulumPlant = LinearSystemId.createSingleJointedArmSystem(
      DCMotor.getFalcon500(motorGearbox.length),
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

  public void zero() {
    double absoluteRotationPosition = Conversions.degreesToFalcon(
      Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition()).getDegrees() - PendulumConstants.cancoderOffset, 
      PendulumConstants.gearing
    );

    for (TalonFXW falcon: motorGearbox) {
      falcon.setSelectedSensorPosition(absoluteRotationPosition);
    }
  }

  public void set(double voltage) {
    for (TalonFXW talon: motorGearbox) {
      talon.setVoltage(voltage);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
