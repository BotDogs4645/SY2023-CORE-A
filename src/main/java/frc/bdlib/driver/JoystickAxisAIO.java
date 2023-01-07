package frc.bdlib.driver;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleUnaryOperator;

import frc.bdlib.misc.BDConstants.JoystickConstants.JoystickAxisID;

public class JoystickAxisAIO {

    public static final DoubleUnaryOperator LINEAR = in -> in;
    public static final DoubleUnaryOperator AGGRESSIVE = in -> ((2.0/3.0) * in) + ((1.0/3.0) * Math.pow(in, 3));
    public static final DoubleUnaryOperator INTERMEDIATE = in -> ((1.0/2.0) * in) + ((1.0/2.0) * Math.pow(in, 3));
    public static final DoubleUnaryOperator GENTLE = in -> ((1.0/3.0) * in) + ((2.0/3.0) * Math.pow(in, 3));
    public static final DoubleUnaryOperator CUBIC = in -> Math.pow(in, 3);

    private final ControllerAIO controller;
    private final JoystickAxisID permanent_id;
    private final DoubleUnaryOperator easing;
    private final double deadzone;

    public JoystickAxisAIO(ControllerAIO controller, JoystickAxisID permanent_id, DoubleUnaryOperator easing, double deadzone) {
        this.controller = controller;
        this.permanent_id = permanent_id;
        this.easing = easing;
        this.deadzone = deadzone;
    }

    public JoystickAxisAIO(ControllerAIO controller, JoystickAxisID permanent_id, DoubleUnaryOperator easing) {
        this(controller, permanent_id, easing, 0.0);
    }

    public JoystickAxisAIO(ControllerAIO controller, JoystickAxisID permanent_id, double deadzone) {
        this(controller, permanent_id, INTERMEDIATE, deadzone);
    }

    public JoystickAxisAIO(ControllerAIO controller, JoystickAxisID permanent_id) {
        this(controller, permanent_id, INTERMEDIATE, 0.0);
    }

    public double getValue() {
        double in = controller.getRawAxis(controller.getVariant().getAxis(permanent_id));

        double value = Math.abs(in) < deadzone ? 0 : in;

        return easing.applyAsDouble(value);
    }

    public BooleanSupplier axisHigherThan(double value) {
        return () -> this.getValue() > value;
    }
}
