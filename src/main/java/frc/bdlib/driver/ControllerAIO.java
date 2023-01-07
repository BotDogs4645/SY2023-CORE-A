package frc.bdlib.driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.bdlib.misc.BDConstants.JoystickConstants.JoystickAxisID;
import frc.bdlib.misc.BDConstants.JoystickConstants.JoystickButtonID;
import frc.bdlib.misc.BDConstants.JoystickConstants.JoystickVariant;

import java.util.EnumMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleUnaryOperator;

public class ControllerAIO extends GenericHID {
    private final JoystickVariant joystickType;
    private final Map<JoystickButtonID, JoystickButton> actualButtons = new EnumMap<>(JoystickButtonID.class);

    public ControllerAIO(int port) {
        super(port);
        Optional<JoystickVariant> found = JoystickVariant.findJoy(DriverStation.getJoystickName(super.getPort()));

        joystickType = found.orElse(JoystickVariant.XBOX);

        for (JoystickButtonID id : JoystickButtonID.values()) {
            actualButtons.put(id, new JoystickButton(this, joystickType.getButton(id)));
        }

    }

    public JoystickButton getJoystickButton(JoystickButtonID id) {
        return actualButtons.get(id);
    }

    public ToggleBooleanSupplier getToggleBooleanSupplier(JoystickButtonID id, double debounce) {
        return new ToggleBooleanSupplier(actualButtons.get(id), debounce);
    }

    public JoystickAxisAIO getAxis(JoystickAxisID id, DoubleUnaryOperator line_function, double deadzone) {
        return new JoystickAxisAIO(this, id, line_function, deadzone);
    }

    public JoystickAxisAIO getAxis(JoystickAxisID id, DoubleUnaryOperator line_function) {
        return new JoystickAxisAIO(this, id, line_function, 0.0);
    }

    public JoystickAxisAIO getAxis(JoystickAxisID id, double deadzone) {
        return new JoystickAxisAIO(this, id, JoystickAxisAIO.INTERMEDIATE, deadzone);
    }

    public JoystickAxisAIO getAxis(JoystickAxisID id) {
        return new JoystickAxisAIO(this, id, JoystickAxisAIO.INTERMEDIATE, 0.0);
    }

    public JoystickVariant getVariant() {
        return joystickType;
    }

    public boolean canRumble() {
        return joystickType.canRumble();
    }
}
