package frc.bdlib.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Provides an actually usable API for Limelight.<br>
 * Snapshot details are in {@link LimelightSnapshot}.
 * @see <a href="https://docs.limelightvision.io/en/latest/networktables_api.html">Limelight NetworkTables API</a>
 */
public class Limelight {

    public static final String LIMELIGHT_TABLE = "limelight";

    /**
     * @return the relevant NetworkTable for Limelight
     */
    public static NetworkTable getTable() {
        return NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE);
    }

    /**
     * Retrieves a snapshot of the Limelight data. Snapshots are useful because staggering your requests across a longer
     * period of time increases the chance of data being written as in-between reads, but they're mainly just convenient.
     * @return the current Limelight snapshot
     */
    public static LimelightSnapshot snapshot() {
        return LimelightSnapshot.of(getTable());
    }


}
