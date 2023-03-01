// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bdlib.custom_talon.TalonFXW;
import frc.robot.Constants.PendulumConstants;

public class Pendulum extends SubsystemBase {
  public static class PendulumControlRequest {
    private Pose2d robotPosition;
    private TrapezoidProfile.State pendulumRotationAngle;

    // simple trig to determine pendulum rotation angle and bot position
    public PendulumControlRequest(Pose3d endEffector) {
      // The equation below graphs a semi-circle:
      // If the arm length is not long enough to reach the desired HEIGHT, then it will return a translation with a NaN X component
      double distanceXFromEndEffector = Math.sqrt(
        Math.pow(PendulumConstants.armLength, 2) - Math.pow(endEffector.getZ() - PendulumConstants.heightOfAxis, 2)
      );

      robotPosition = new Pose2d(
        new Translation2d(endEffector.getX() + distanceXFromEndEffector, endEffector.getY()),
        Rotation2d.fromRadians(0)
      );

      pendulumRotationAngle = new TrapezoidProfile.State(
        Math.asin((-PendulumConstants.heightOfAxis + endEffector.getZ()) / PendulumConstants.armLength),
        0);
    }

    public Pose2d getRobotPosition() {
      return robotPosition;
    }

    public TrapezoidProfile.State getPendulumRotation() {
      return pendulumRotationAngle;
    }
  }

  private ArmFeedforward arm;
  private ProfiledPIDController pid;

  private TalonFXW plantMotor;
  private TalonFXW followerMotor;
  private CANCoder absoluteEncoder;

  private ShuffleboardTab tab;
  
  /** Creates a new PIDPendulum. */
  public Pendulum() {
    this.plantMotor = new TalonFXW(PendulumConstants.controllerId, "", PendulumConstants.pendulumFalconsConfig);
    this.followerMotor = new TalonFXW(PendulumConstants.followerId, "", PendulumConstants.pendulumFalconsConfig);

    this.absoluteEncoder = new CANCoder(PendulumConstants.cancoderId);
    // we go -180 to 180 to represent the negative as below the horizon
    absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    absoluteEncoder.setPositionToAbsolute();

    this.arm = new ArmFeedforward(0.32005, 0.54473, 1.3389, 0.19963);
    this.pid = new ProfiledPIDController(
      14.255,
      0,
      7.2173, 
      new TrapezoidProfile.Constraints(2, 1)
    );

    plantMotor.setInverted(TalonFXInvertType.CounterClockwise);
    followerMotor.setInverted(TalonFXInvertType.Clockwise);

    plantMotor.setNeutralMode(NeutralMode.Brake);
    followerMotor.setNeutralMode(NeutralMode.Brake);

    followerMotor.follow(plantMotor);

    followerMotor.setInverted(TalonFXInvertType.OpposeMaster);
    pid.setTolerance(Math.toRadians(2), Math.toRadians(0.1));

    pid.reset(new TrapezoidProfile.State(getPendulumPosition(), getPendulumVelocity()));

    this.tab = Shuffleboard.getTab("Arm");
    tab.addNumber("Arm Absolute Angle (degrees)", () -> absoluteEncoder.getAbsolutePosition());
    tab.addNumber("Arm velo (degrees / second)", () -> absoluteEncoder.getVelocity());
    tab.addNumber("Arm error (degrees)", () -> pid.getPositionError() * (180 / Math.PI));
    tab.addNumber("Arm left amp", () -> plantMotor.getStatorCurrent());
    tab.addNumber("Arm right amp", () -> followerMotor.getStatorCurrent());
    tab.add(pid);

    zero();
  }

  public void move(TrapezoidProfile.State angle) {
    plantMotor.setVoltage(
      pid.calculate(getPendulumPosition(), angle) +
      arm.calculate(angle.position, angle.velocity)
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getPendulumPosition() {
    return absoluteEncoder.getAbsolutePosition() * (Math.PI / 180.0);
  }

  public void zero() {
    pid.reset(getPendulumVelocity(), getPendulumPosition());
  }

  public double getPendulumVelocity() {
    return absoluteEncoder.getVelocity() * (Math.PI / 180.0);
  }

  public double getError() {
    return pid.getPositionError();
  }

  public boolean atSetpoint() {
    return pid.atSetpoint();
  }
}