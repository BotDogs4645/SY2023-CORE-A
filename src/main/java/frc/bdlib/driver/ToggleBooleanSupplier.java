package frc.bdlib.driver;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ToggleBooleanSupplier {
    private boolean actualValue = false;
    private double timeLastPressed = System.currentTimeMillis();

    public ToggleBooleanSupplier(JoystickButton button, double debounce) {
        new RunCommand(() -> {
            if (button.getAsBoolean() && (System.currentTimeMillis() - timeLastPressed) / 1000 >= debounce) {
                actualValue = !actualValue;
                timeLastPressed = System.currentTimeMillis();
            }
        })
        .ignoringDisable(true)
        .schedule();
    }

    public boolean getValue() {
        return actualValue;
    }
}
