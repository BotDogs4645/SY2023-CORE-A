package frc.bdlib.misc;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class BDManager {
    private static BDManager _singleton;

    public static BDManager getInstance() {
        if (_singleton != null) {
            return _singleton;
        } else {
            DriverStation.reportError("BDUpdateManager is not initialized", false);
            return null;
        }
    }

    public static boolean isInstantiated() {
        return _singleton != null;
    }

    public static void initialize() {
        if (_singleton == null) {
            _singleton = new BDManager();
        }
    }

    private final List<BDUpdatable> updateList;
    private final ShuffleboardTab tab;
    private final ShuffleboardLayout statusLayout;

    private BDManager() {
        updateList = new ArrayList<>();
        tab = Shuffleboard.getTab("BDManager");
        statusLayout = tab.getLayout("BD Device Statuses", BuiltInLayouts.kGrid);
        statusLayout.withSize(4, 4)
            .withPosition(0, 0);
    }

    public void register(BDUpdatable item, boolean reportStatus) {
        if (isInstantiated()) {
            updateList.add(item);
            if (reportStatus) {
                statusLayout.addString(item.getID(), item::getStatus);
            }
        }
    }

    public ShuffleboardTab getInstanceManagerialTab() {
        if (isInstantiated()) {
            return tab;
        } else {
            DriverStation.reportError("Cannot access BDManager shuffleboard tab w/o being instantiated first.", false);
            return null;
        }
    }

    public void update() {
        for (BDUpdatable item : updateList) {
            item.update();
        }
    }
}