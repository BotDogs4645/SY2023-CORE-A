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
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.bdlib.custom_talon.TalonFXW;
import frc.robot.Constants.PendulumConstants;

public class Pendulum extends ProfiledPIDSubsystem {
  public static class PendulumControlRequest {
    private Pose2d robotPosition;
    private TrapezoidProfile.State pendulumRotationAngle;

    // simple trig to determine pendulum rotation angle and bot position
    public PendulumControlRequest(Pose3d endEffector) {
      // The equation below graphs a semi-circle:
      // If the arm length is not long enough to reach the desired HEIGHT, then it
      // will return a translation with a NaN X component
      double distanceXFromEndEffector = Math.sqrt(
          Math.pow(PendulumConstants.armLength, 2) - Math.pow(endEffector.getZ() - PendulumConstants.heightOfAxis, 2));

      robotPosition = new Pose2d(
          new Translation2d(endEffector.getX() + distanceXFromEndEffector, endEffector.getY()),
          Rotation2d.fromRadians(0));

      pendulumRotationAngle = new TrapezoidProfile.State(
          Math.asin((-PendulumConstants.heightOfAxis + endEffector.getZ()) / PendulumConstants.armLength) + Math.toRadians(9), // gear ratio offset
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

  private double pidWant;
  private double ffWant;

  private TalonFXW plantMotor;
  private TalonFXW followerMotor;
  private CANCoder absoluteEncoder;

  public double wantedAngle;
  private double offset;
 
  private ShuffleboardTab tab;

  /** Creates a new PendulumProfiled. */
  public Pendulum() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            8.25,
            0,
            0.05,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(1.5, 0.7)));

    this.plantMotor = new TalonFXW(PendulumConstants.controllerId, "", PendulumConstants.pendulumFalconsConfig);
    this.followerMotor = new TalonFXW(PendulumConstants.followerId, "", PendulumConstants.pendulumFalconsConfig);

    this.absoluteEncoder = new CANCoder(PendulumConstants.cancoderId);
    // we go -180 to 180 to represent the negative as below the horizon
    absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    absoluteEncoder.setPositionToAbsolute();

    this.arm = new ArmFeedforward(0.48005, 0.54473, 1.3389, 0.19963);
    this.ffWant = 0;

    plantMotor.setInverted(TalonFXInvertType.CounterClockwise);
    followerMotor.setInverted(TalonFXInvertType.Clockwise);

    plantMotor.setNeutralMode(NeutralMode.Brake);
    followerMotor.setNeutralMode(NeutralMode.Brake);

    followerMotor.follow(plantMotor);

    followerMotor.setInverted(TalonFXInvertType.OpposeMaster);

    super.getController().setTolerance(1.5, .5);

    this.tab = Shuffleboard.getTab("Arm");
    tab.addNumber("Arm Absolute Angle (degrees)", () -> absoluteEncoder.getAbsolutePosition());
    tab.addNumber("Arm velo (degrees / second)", () -> absoluteEncoder.getVelocity());
    tab.addNumber("Arm left amp", () -> plantMotor.getStatorCurrent());
    tab.addNumber("Arm right amp", () -> followerMotor.getStatorCurrent());
    tab.addNumber("Arm left volts", () -> plantMotor.getMotorOutputVoltage());
    tab.addNumber("Arm right volts", () -> followerMotor.getMotorOutputVoltage());

    tab.addNumber("PID Want", () -> getPIDVoltage());
    tab.addNumber("FF want", () -> getFFVoltage());

    this.offset = absoluteEncoder.configGetMagnetOffset();

    tab.add(this);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    ffWant = arm.calculate(setpoint.position, setpoint.velocity);

    plantMotor.setVoltage(output + ffWant);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getPendulumPosition();
  }

  public void zeroPendulum() {
    double currentValue = absoluteEncoder.configGetMagnetOffset();
    double absPos = absoluteEncoder.getAbsolutePosition();
    absoluteEncoder.configMagnetOffset(currentValue - absPos);
  }

  public double getPendulumPosition() {
    return absoluteEncoder.getAbsolutePosition() * (Math.PI / 180.0);
  }

  public double getPendulumVelocity() {
    return absoluteEncoder.getVelocity() * (Math.PI / 180.0);
  }

  public double getError() {
    return super.getController().getPositionError();
  }

  public boolean atSetpoint() {
    return super.getController().atSetpoint();
  }

  public double getPIDVoltage() {
    return pidWant;
  }

  public void setWantedAngle(double wantedAngle) {
    this.wantedAngle = wantedAngle;
  }

  public void changeOffset(double povIn) {
    if (povIn == 180) {
      offset += 0.2;
      absoluteEncoder.configMagnetOffset(offset);
    } else if (povIn == 0) {
      offset -= 0.2;
      absoluteEncoder.configMagnetOffset(offset);
    }
  }

  public double getFFVoltage() {
    return ffWant;
  }
}