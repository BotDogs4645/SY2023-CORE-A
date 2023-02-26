// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.bdlib.custom_talon.TalonFXW;
import frc.bdlib.misc.BDConstants.JoystickConstants.JoystickButtonID;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean testing = true;
    public static final NetworkTable LogTable = NetworkTableInstance.getDefault().getTable("Loggables");

    public static class PendulumConstants {
        // picture for reference
        //
        public static enum PendulumCommand {
            IdleWithPiece(-Math.PI / 4),
            Idle(-Math.PI / 2),
            Straight(0),
            RetrieveAngle(0), // TODO: find retrieve angle
            ;

            private double angle;

            private PendulumCommand(double position) {
                this.angle = position;
            }

            public double get() {
                return angle;
            }
        }

        public static final TalonFXW.FXWConfig pendulumFalconsConfig = new TalonFXW.FXWConfig(48, 0);
        public static final int numOfMotors = 2;

        // TODO: find ids
        public static final int controllerId = 14;
        public static final int followerId = 15;

        public static final int cancoderId = 16;
        public static final double cancoderOffset = 25;

        /* State Space Settings */
        public static double measurementDelay = 0.10339;

        // Characterization
        public static final double kS = 0.084424;
        public static final double kV = 0.67768;
        public static final double kA = 0.13784;

        // Other
        public static final double momentOfInertia = 17.43589;
        public static final double gearing = 48.0 / 1.0; // output / input
        public static final TrapezoidProfile.Constraints pendulumConstraints = new TrapezoidProfile.Constraints(
                Units.degreesToRadians(45),
                Units.degreesToRadians(90)); // Max arm speed and acceleration.

        public static final double heightOfAxis = Units.inchesToMeters(50.847); // from Drew
        public static final double armLength = Units.inchesToMeters(45.011);

    }

    public static class CameraConstants {
        // "x+" = Pigeon2 orientation dependent ;p - check which direction points
        // forward.. x+ can be technically defined as the x+ from a pose
        // defined from the origin of the robot chassis facing the "front face" of the
        // robot. Or atleast it should be.
        public static enum CameraDefaults {
            MountOne(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)),
            MountTwo(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));

            Transform3d overallTransform;

            private CameraDefaults(Translation3d translation, Rotation3d rotation) {
                this.overallTransform = new Transform3d(translation, rotation);
            }

            public Transform3d getTransformation() {
                return overallTransform;
            }
        }
    }

    public static class AutoPositionConstants {
        public static enum AprilTagTransformDirection {
            // note: no rotation transformation should be required
            // this direction should only move the apriltag pose left, center, or right, so
            // the Y coord
            // That coord should basically be aligned w/ the apriltag pose in the Z
            // dimension
            // just translated to the left, center or right.
            // found from CAD btw

            // Transforms the april tag to a position to the..
            LEFT(new Translation3d(0, Units.inchesToMeters(22), 0), new Rotation3d()),
            CENTER(new Translation3d(0, 0, 0), new Rotation3d()),
            RIGHT(new Translation3d(0, Units.inchesToMeters(-22), 0), new Rotation3d());

            Transform3d transform;

            private AprilTagTransformDirection(Translation3d translate, Rotation3d heading) {
                this.transform = new Transform3d(translate, heading);
            }

            public Transform3d getTransform() {
                return transform;
            }
        }

        public static enum GamePiecePlacementLevel {
            BOTTOM(new Translation3d(Units.inchesToMeters(6.77500), 0, Units.inchesToMeters(-14.53)), new Rotation3d()),
            MIDDLE(new Translation3d(Units.inchesToMeters(-8.9125), 0, Units.inchesToMeters(17.5)), new Rotation3d()),
            TOP(new Translation3d(Units.inchesToMeters(-25.9256), 0, Units.inchesToMeters(29.845)), new Rotation3d());

            Transform3d transform;

            private GamePiecePlacementLevel(Translation3d translate, Rotation3d heading) {
                this.transform = new Transform3d(translate, heading);
            }

            public Transform3d getTransform() {
                return transform;
            }
        }

        public static Map<JoystickButtonID, GamePiecePlacementLevel> placementButtons = Map.of(
                JoystickButtonID.kA, GamePiecePlacementLevel.BOTTOM,
                JoystickButtonID.kX, GamePiecePlacementLevel.MIDDLE,
                JoystickButtonID.kB, GamePiecePlacementLevel.MIDDLE,
                JoystickButtonID.kY, GamePiecePlacementLevel.TOP);

        public static AprilTagTransformDirection getPoseAlignmentFromNumber(int number) {
            return switch (number) {
                case 270 -> AprilTagTransformDirection.LEFT;
                case 0 -> AprilTagTransformDirection.CENTER;
                case 90 -> AprilTagTransformDirection.RIGHT;

                default -> AprilTagTransformDirection.CENTER;
            };
        }
    }
}