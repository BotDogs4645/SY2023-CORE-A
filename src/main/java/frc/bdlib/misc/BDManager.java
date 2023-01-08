package frc.bdlib.misc;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class BDManager {
    static private BDManager _singleton;
    static private boolean _started = false;

    private List<BDUpdatable> updateList;
    private ShuffleboardTab tab;
    private ShuffleboardLayout statusLayout;

    public static BDManager getInstance() {
        if (_singleton != null) {
            return _singleton;
        } else {
            DriverStation.reportError("BDUpdateManager is not initialized", false);
            return null;
        }
    }

    public static boolean isInstantiated() {
        return _started;
    }

    public static void initialize() {
        if (!_started) {
            _singleton = new BDManager();
            _started = true;
        }
    }

    private BDManager() {
        updateList = new ArrayList<>();
        tab = Shuffleboard.getTab("BDManager");
        statusLayout = tab.getLayout("BD Device Statuses", BuiltInLayouts.kGrid);
        statusLayout.withSize(4, 4)
                .withPosition(0, 0);
    }

    public void register(BDUpdatable item) {
        if (isInstantiated()) {
            updateList.add(item);
            statusLayout.addString(item.getID(), item::getStatus);
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