package frc.bdlib.limelight;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;

/**
 * A snapshot of Limelight information. This <i>should</i> be completely immutable (including fields).<br>
 * Note: this class does not manage the untapped potential of the JSON dump, simply providing a view into the
 * string; there is a lot of untapped information.<br>
 * A large amount of the data contained here may be invalid in some way if there isn't a valid target, so checking that
 * there is a valid target before using this is imperative!!
 * @param target the detected target (possibly null) - see {@link Target}
 * @param bounds the bounds of the target - see {@link BoundingBox}
 * @param pipelineResult some information about the pipeline output - see {@link PipelineResult}
 * @param cameraTransform the transform of the camera in the target's space
 * @param robotTransform the absolute transform of the robot (well, relative to the field)
 * @param jsonResultDump Limelight's complete dump of the latest targeting information - see the
 *                       <a href="https://docs.limelightvision.io/en/latest/json_dump.html">JSON dump specification</a>
 */
public record LimelightSnapshot(Target target, BoundingBox bounds, PipelineResult pipelineResult,
                       Transform3d cameraTransform, Transform3d robotTransform,
                       String jsonResultDump) {

    public static LimelightSnapshot of(NetworkTable table) {
        boolean hasTarget = table.getEntry("tv").getInteger(0L) != 0;
        return new LimelightSnapshot(
                hasTarget ? Target.snapshot(table) : null,
                BoundingBox.snapshot(table),
                PipelineResult.snapshot(table),
                // For some reason they apparently messed up the rotation axis order, so this is necessary
                buildTransform(table.getEntry("camtran").getDoubleArray(new double[0]), 4, 5, 3),
                buildTransform(table.getEntry("botpose").getDoubleArray(new double[0]), 3, 4, 5),
                table.getEntry("json").getString(null)
        );
    }

    public boolean hasValidTarget() {
        return target != null;
    }

    /**
     * Stores general target information for a snapshot.
     * @param xOffset the horizontal offset from the crosshair to the target - supposedly measured in degrees
     * @param yOffset the vertical offset from the crosshair to the target - also supposedly measured in degrees
     * @param area how much of the screen this target occupies, measured as the percentage of the screen
     * @param rotation the skew or rotation of this target, in degrees - ranges from -90 to 0 degrees
     */
    public record Target(double xOffset, double yOffset,
                         double area, double rotation) {

        public static Target snapshot(NetworkTable table) {
            return new Target(
                    table.getEntry("tx").getDouble(Double.NaN),
                    table.getEntry("ty").getDouble(Double.NaN),
                    table.getEntry("ta").getDouble(Double.NaN),
                    table.getEntry("ts").getDouble(Double.NaN)
            );
        }

    }

    /**
     * Stores general information about the alleged bounding box of the target
     * @param fittedShortestSide the length of the shortest side on the fitted bounding box, in pixels
     * @param fittedLongestSide the length of the longest side on the fitted bounding box, in pixels
     * @param roughWidth horizontal side length of the rough bounding box, in pixels - ranges from 0 to 320 pixels
     * @param roughHeight vertical side length of the rough bounding box, in pixels - ranges from 0 to 320 pixels
     */
    public record BoundingBox(double fittedShortestSide, double fittedLongestSide,
                              double roughWidth, double roughHeight) {

        public static BoundingBox snapshot(NetworkTable table) {
            return new BoundingBox(
                    table.getEntry("tshort").getDouble(Double.NaN),
                    table.getEntry("tlong").getDouble(Double.NaN),
                    table.getEntry("thor").getDouble(Double.NaN),
                    table.getEntry("tvert").getDouble(Double.NaN)
            );
        }

    }

    /**
     *
     * @param latency the contribution to the latency made by the pipeline, in milliseconds. Limelight suggests adding
     *                at least 11ms to make up for the image capture latency
     * @param activePipeline which camera pipeline is currently active. Ranges from 0 to 9.
     * @param detectedAprilTag the fiducial ID of the detected AprilTag.
     * @param neutralDetectorClass class ID of the neural detector/classifier result
     */
    public record PipelineResult(double latency, long activePipeline,
                                 long detectedAprilTag, long neutralDetectorClass) {

        public static PipelineResult snapshot(NetworkTable table) {
            return new PipelineResult(
                    table.getEntry("tl").getDouble(Double.NaN),
                    table.getEntry("getpipe").getInteger(-1),
                    table.getEntry("tid").getInteger(Integer.MIN_VALUE), // Because april tags usually encode less than 12 bits of data, this should be safe
                    table.getEntry("tclass").getInteger(-1)
            );
        }

    }

    private static Transform3d buildTransform(double[] nums, int roll, int pitch, int yaw) {
        var translation = new Translation3d(nums[0], nums[1], nums[2]);
        var rotation = new Rotation3d(nums[roll], nums[pitch], nums[yaw]);

        return new Transform3d(translation, rotation);
    }

}