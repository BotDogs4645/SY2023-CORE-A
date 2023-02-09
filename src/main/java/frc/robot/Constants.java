// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.bdlib.custom_talon.TalonFXW;
import frc.bdlib.misc.BDConstants.JoystickConstants.JoystickButtonID;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean testing = true;
    public static final NetworkTable LogTable = NetworkTableInstance.getDefault().getTable("Loggables");

    public static class PendulumConstants {
        public static final TalonFXW.FXWConfig pendulumFalconsConfig = new TalonFXW.FXWConfig(0, 0);
        public static final int numOfMotors = 3;

        // TODO: find ids
        public static final int controllerId = 1;
        public static final int followerId = 2;

        public static final int cancoderId = 0;
        public static final double cancoderOffset = 0.0;

        /* State Space Settings */

        // Characterization
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;      
        
        // Other
        public static final double momentOfInertia = 0.0;
        public static final double gearing = 1.0 / 1.0; // output / input
        private final TrapezoidProfile.Constraints m_constraints =
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(45),
                Units.degreesToRadians(90)
            ); // Max arm speed and acceleration.
    }

    public static class CameraConstants {
        // "x+" = Pigeon2 orientation dependent ;p - check which direction points forward.. x+ can be technically defined as the x+ from a pose
        // defined from the origin of the robot chassis facing the "front face" of the robot. Or atleast it should be.
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
            // Transforms the april tag to a position to the..
            LEFT(new Translation2d(), new Rotation2d()),
            CENTER(new Translation2d(), new Rotation2d()),
            RIGHT(new Translation2d(), new Rotation2d()); 

            Transform2d transform;
            private AprilTagTransformDirection(Translation2d translate, Rotation2d heading) {
                this.transform = new Transform2d(translate, heading);
            }

            public Transform2d getTransform() {
                return transform;
            }
        }

        public static enum GamePiecePlacementLevel {
            BOTTOM(),
            MIDDLE(),
            TOP()
            ;

            /* args depend on what method we use to determine what type of inputs we grab here.
             * For example, if we:
             * do an arm: These will be Pose3ds that the end affector has to move to.
             * do an elevator with telescoping arm: These will be xz coordinates with a Y scalar to represent length
             * other stuff: idk lol
             */
            private GamePiecePlacementLevel() {

            }
        }

        public static Map<JoystickButtonID, GamePiecePlacementLevel> placementButtons = Map.of(
            JoystickButtonID.kA, GamePiecePlacementLevel.BOTTOM,
            JoystickButtonID.kX, GamePiecePlacementLevel.MIDDLE,
            JoystickButtonID.kY, GamePiecePlacementLevel.TOP
        );

        public static AprilTagTransformDirection getPoseAlignmentFromNumber(int number) {
            return switch (number) {
                case 270 -> AprilTagTransformDirection.LEFT;
                case 0 -> AprilTagTransformDirection.CENTER;
                case 90 -> AprilTagTransformDirection.RIGHT;

                default -> AprilTagTransformDirection.CENTER;
            };
        }
    }
    // 
    public static final double FIELD_ZERO_MAGNETIC_HEADING = 0;


    public static enum AlignedPose {
        ;
        
        Translation2d aligned_pose;
        private AlignedPose(double x, double y) {
            aligned_pose = new Translation2d(x, y);
        }
    }
}