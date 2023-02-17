// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.bdlib.misc.BDConstants.JoystickConstants.JoystickButtonID;

import java.util.Map;

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

    public static class ClawConstants {

        public static final int motorDeviceId = 1; // TODO: Get actual value

        public static final double closeSpeed = -0.25; // TODO: Calibrate a good value
        public static final double openSpeed = 0.25; // TODO: Calibrate a good value

        public static final JoystickButtonID buttonOpen = null; // TODO: Find a good button
        public static final JoystickButtonID buttonClose = null; // TODO: Find a good button

        public static final int limitSwitchChannel = 1; // TODO: Get actual value

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