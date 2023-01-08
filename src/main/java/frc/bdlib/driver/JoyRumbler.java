package frc.bdlib.driver;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JoyRumbler extends SubsystemBase {
    public enum RumblerType {
        LEFT_SHAKER,
        RIGHT_SHAKER
    }

    Map<RumblerType, ArrayList<BooleanSupplier>> shakers = new HashMap<>();
    ControllerAIO xbox;
    boolean analysis_mode = true;

    BooleanSupplier specific_supplier;
    ToggleBooleanSupplier muter;

    public JoyRumbler(ControllerAIO xbox, ToggleBooleanSupplier muter) {
        this.xbox = xbox;
        this.muter = muter;
        for (RumblerType enu: RumblerType.values()) {
            shakers.put(enu, new ArrayList<>());
        }
        xbox.setRumble(RumbleType.kBothRumble, 1.0);
    }

    public void addRumbleScenario(BooleanSupplier suppl, RumblerType rumble_type) {
        shakers.get(rumble_type).add(suppl);
    }

    public void removeRumbleScenario(RumblerType type, BooleanSupplier suppl) {
        shakers.get(type).remove(suppl);
    }

    public void setSpecificRumble(BooleanSupplier suppl) {
        analysis_mode = false;
        if (this.getCurrentCommand() != null) {
            this.getCurrentCommand().cancel();
        }
        specific_supplier = suppl;
    }

    public void disableSpecificRumble() {
        analysis_mode = true;
        xbox.setRumble(RumbleType.kLeftRumble, 0);
        xbox.setRumble(RumbleType.kRightRumble, 0);
    }
    
    @Override
    public void periodic() {
        if (!muter.getValue()) {
            xbox.setRumble(RumbleType.kBothRumble, 0);
        }

        boolean left, right;

        if (analysis_mode) {
            left = shakers.get(RumblerType.LEFT_SHAKER).stream().anyMatch(BooleanSupplier::getAsBoolean);
            right = shakers.get(RumblerType.RIGHT_SHAKER).stream().anyMatch(BooleanSupplier::getAsBoolean);
        } else {
            boolean actualValue = specific_supplier.getAsBoolean();
            left = actualValue;
            right = actualValue;
        }

        xbox.setRumble(RumbleType.kLeftRumble, left ? 1 : 0);
        xbox.setRumble(RumbleType.kRightRumble, right ? 1 : 0);
    }
}
