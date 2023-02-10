// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bdlib.custom_talon.TalonFXW;
import frc.robot.Constants.PendulumConstants;
import frc.robot.util.swervehelper.Conversions;

public class Pendulum extends SubsystemBase {
  public static record PendulumControlRequest(Pose3d endEffector) {
    private static Pose2d robotPosition;
    private static TrapezoidProfile.State pendulumRotationAngle;

    // simple trig to determine pendulum rotation angle and bot position
    public PendulumControlRequest {
      double distanceXFromEndEffector = Math.sqrt(
        Math.pow(PendulumConstants.armLength, 2) - Math.pow(endEffector.getZ() - PendulumConstants.heightOfAxis, 2)
      );

      robotPosition = new Pose2d(
        new Translation2d(endEffector.getX() + distanceXFromEndEffector, endEffector.getY()),
        Rotation2d.fromRadians(-Math.PI)
      );

      pendulumRotationAngle = new TrapezoidProfile.State(Math.asin(endEffector.getZ() - PendulumConstants.heightOfAxis / PendulumConstants.armLength), 0);
    }

    public Pose2d getRobotPosition() {
      return robotPosition;
    }

    public TrapezoidProfile.State getPendulumRotation() {
      return pendulumRotationAngle;
    }
  }

  private TalonFXW plantMotor;
  private TalonFXW followerMotor;
  private CANCoder absoluteEncoder;

  /* State space components */
  private LinearSystem<N2, N1, N1> pendulumPlant;
  private KalmanFilter<N2, N1, N1> observer;
  private LinearQuadraticRegulator<N2, N1, N1> LQR;

  private TrapezoidProfile.State lastProfiledReference;
  private LinearSystemLoop<N2, N1, N1> controlLoop;

  public Pendulum() {
    /* Motor declarations - keep it CTRE && on RIO can bus ("") */
    this.plantMotor = new TalonFXW(PendulumConstants.controllerId, "", PendulumConstants.pendulumFalconsConfig);
    this.followerMotor = new TalonFXW(PendulumConstants.followerId, "", PendulumConstants.pendulumFalconsConfig);

    plantMotor.setInverted(TalonFXInvertType.CounterClockwise);
    followerMotor.setInverted(TalonFXInvertType.Clockwise);

    this.absoluteEncoder = new CANCoder(PendulumConstants.cancoderId);
    // we go -180 to 180 to represent the negative as below the horizon
    absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    /* State space stuff */
    // Pendulum arm model
    this.pendulumPlant = LinearSystemId.createSingleJointedArmSystem(
      DCMotor.getFalcon500(1),
      PendulumConstants.momentOfInertia,
      PendulumConstants.gearing
    );

    // Observer. Rejects noise as much as possible using a KalmanFilter
    this.observer = new KalmanFilter<>(
      Nat.N2(),
      Nat.N1(),
      pendulumPlant,
      VecBuilder.fill(0.015, 0.17), // model accuracy std dev.
      VecBuilder.fill(0.001), // encoder accuracy std dev. low because we trust it
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
    zero();

    lastProfiledReference = new TrapezoidProfile.State(plantMotor.getShaftRotationsInRadians(), plantMotor.getShaftVelocityInRadians());
  }

  public void zero() {
    double absoluteRotationPosition = Conversions.degreesToFalcon(
      Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition()).getDegrees() - PendulumConstants.cancoderOffset, 
      PendulumConstants.gearing
    );
    plantMotor.setSelectedSensorPosition(absoluteRotationPosition);
    followerMotor.setSelectedSensorPosition(absoluteRotationPosition);

    controlLoop.reset(
      VecBuilder.fill(
        plantMotor.getShaftRotationsInRadians(),
        plantMotor.getShaftVelocityInRadians()
      )
    );

    lastProfiledReference = new TrapezoidProfile.State(plantMotor.getShaftRotationsInRadians(), plantMotor.getShaftVelocityInRadians());
  }

  public void set(double voltage) {
    plantMotor.setVoltage(voltage);
    followerMotor.setVoltage(voltage); 
  }

  public void move(TrapezoidProfile.State angle) {
    lastProfiledReference = 
      (new TrapezoidProfile(PendulumConstants.pendulumConstraints, angle, lastProfiledReference)).calculate(0.020);
    controlLoop.setNextR(lastProfiledReference.position, lastProfiledReference.velocity);

    controlLoop.correct(VecBuilder.fill(plantMotor.getShaftRotationsInRadians()));

    controlLoop.predict(0.020);

    this.set(controlLoop.getU(0));
    // TODO: determine if zero() should be called everytime we run a movement cmd
    // might not be required cuz the defaultcommand is to move, so the model is 
    // updated no matter what.. but still. 
  }
  
  @Override
  public void periodic() {}
}
