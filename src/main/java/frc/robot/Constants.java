// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.EnumMap;
import java.util.List;
import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

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
        public static double maxVelo = 1;
        public static double accel = 2;
        public static double timetoVelocity = maxVelo / accel;

        public static enum AlignmentDirection {
            LEFT,
            CENTER,
            RIGHT
        }

        // TODO: decide if the rotation2d portion should be magnetic or not
        public static enum AprilTagDerivedPosition {
            PoseClosest(List.of(1, 2), List.of(
                new Pose2d(new Translation2d(0,0), new Rotation2d(0))
            ));

            public static Optional<AprilTagDerivedPosition> getEnumFromID(Integer id) {
                for (AprilTagDerivedPosition position : AprilTagDerivedPosition.values()) {
                    if (position.getIds().contains(id)) {
                        return Optional.of(position);
                    }
                }
                return Optional.empty();
            }

            EnumMap<AlignmentDirection, Pose2d> storage = new EnumMap<>(AlignmentDirection.class);

            private List<Integer> ids;

            private AprilTagDerivedPosition(List<Integer> ids, List<Pose2d> poses) {
                this.ids = ids;
                for (int i = 0; i < storage.size(); i++) {
                    storage.put(AlignmentDirection.values()[i], poses.get(i));
                }
            }

            public List<Integer> getIds() {
                return ids;
            }

            public Pose2d getDirectionalPose(AlignmentDirection dir) {
                return storage.get(dir);
            }
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