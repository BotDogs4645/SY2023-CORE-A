// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bdlib.custom_talon.TalonFXW;
import frc.robot.Constants.PendulumConstants;

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
        Rotation2d.fromRadians(0)
      );

      pendulumRotationAngle = new TrapezoidProfile.State(
        MathUtil.clamp(
          Math.asin((endEffector.getZ() - PendulumConstants.heightOfAxis) / PendulumConstants.armLength),
          -Math.PI / 2,
          Math.PI * 2 / 9
        ),
        0);
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

  private ShuffleboardTab tab;

  public Pendulum() {
    /* Motor declarations - keep it CTRE && on RIO can bus ("") */
    this.plantMotor = new TalonFXW(PendulumConstants.controllerId, "", PendulumConstants.pendulumFalconsConfig);
    this.followerMotor = new TalonFXW(PendulumConstants.followerId, "", PendulumConstants.pendulumFalconsConfig);

    plantMotor.setInverted(TalonFXInvertType.CounterClockwise);
    followerMotor.setInverted(TalonFXInvertType.Clockwise);

    plantMotor.setNeutralMode(NeutralMode.Brake);
    followerMotor.setNeutralMode(NeutralMode.Brake);

    // plantMotor.configStatorCurrentLimit(
    //   new StatorCurrentLimitConfiguration(true, 18.5, 20.0, 2.5)
    // );

    followerMotor.follow(plantMotor);

    followerMotor.setInverted(TalonFXInvertType.OpposeMaster);

    this.absoluteEncoder = new CANCoder(PendulumConstants.cancoderId);
    // we go -180 to 180 to represent the negative as below the horizon
    absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    absoluteEncoder.setPositionToAbsolute();

    /* State space stuff */
     this.pendulumPlant = LinearSystemId.createSingleJointedArmSystem(
       DCMotor.getFalcon500(2),
       PendulumConstants.momentOfInertia,
       PendulumConstants.gearing
    );

    // Observer. Rejects noise as much as possible using a KalmanFilter
    this.observer = new KalmanFilter<>(
      Nat.N2(),
      Nat.N1(),
      pendulumPlant,
      VecBuilder.fill(0.01, 0.12), // model accuracy std dev.
      VecBuilder.fill(0.0001), // encoder accuracy std dev. low because we trust it
      0.020
    );
    
    this.LQR = new LinearQuadraticRegulator<>(
      pendulumPlant,
      VecBuilder.fill(Units.degreesToRadians(2.0), Units.degreesToRadians(5.0)), // qelms.
      // Position and velocity error tolerances, in radians and radians per second. Decrease
      // this to more heavily penalize state excursion, or make the controller behave more
      // aggressively.
      VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
      // heavily penalize control effort, or make the controller less aggressive.
      0.020
    );

    this.controlLoop = new LinearSystemLoop<>(pendulumPlant, LQR, observer, 12.0, 0.020);
    //LQR.latencyCompensate(pendulumPlant, 0.020, PendulumConstants.measurementDelay);
    zero();

    lastProfiledReference = new TrapezoidProfile.State(getPendulumPosition(), getPendulumVelocity());

    this.tab = Shuffleboard.getTab("Arm");
    tab.addNumber("Arm Absolute Angle (degrees)", () -> absoluteEncoder.getAbsolutePosition());
    tab.addNumber("Arm velo (degrees / second)", () -> absoluteEncoder.getVelocity());
    tab.addNumber("Arm error (degrees)", () -> getError() * (180 / Math.PI));
    tab.addNumber("Arm input (volts)", () -> getInput());
    tab.addNumber("Arm output (volts)", () -> getCurrent());
    tab.addNumber("motor left amps", () -> plantMotor.getStatorCurrent());
    tab.addNumber("motor right amps", () -> followerMotor.getStatorCurrent());
    tab.add(this);
  }

  public void zero() {
    controlLoop.reset(
      VecBuilder.fill(
        getPendulumPosition(),
        getPendulumVelocity()
      )
    );

    lastProfiledReference = new TrapezoidProfile.State(getPendulumPosition(), getPendulumVelocity());
  }

  public void set(double voltage) {
    plantMotor.setVoltage(voltage);
  }

  public void move(TrapezoidProfile.State angle) {
    lastProfiledReference = 
      (new TrapezoidProfile(PendulumConstants.pendulumConstraints, angle, lastProfiledReference)).calculate(0.020);
    controlLoop.setNextR(lastProfiledReference.position, lastProfiledReference.velocity);

    controlLoop.correct(VecBuilder.fill(getPendulumPosition()));

    controlLoop.predict(0.020);

    this.set(controlLoop.getU(0));
    // TODO: determine if zero() should be called everytime we run a movement cmd
    // might not be required cuz the defaultcommand is to move, so the model is 
    // updated no matter what.. but still. 
  }

  public double getError() {
    return controlLoop.getError(0);
  }

  public double getInput() {
    return controlLoop.getU(0);
  }

  public double getCurrent() {
    return controlLoop.getXHat(0);
  }
  
  public double getPendulumPosition() {
    return absoluteEncoder.getAbsolutePosition() * (Math.PI / 180.0);
  }

  public double getPendulumVelocity() {
    return absoluteEncoder.getVelocity() * (Math.PI / 180.0);
  }

  @Override
  public void periodic() {}
}
