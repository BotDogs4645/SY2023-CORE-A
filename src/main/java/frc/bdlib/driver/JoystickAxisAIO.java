package frc.bdlib.driver;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleUnaryOperator;

import frc.bdlib.misc.BDConstants.JoystickConstants.JoystickAxisID;

public record JoystickAxisAIO(
        ControllerAIO controller,
        JoystickAxisID permanentId,
        DoubleUnaryOperator easing,
        double deadzone
    ) {

    public record AxisSettings(DoubleUnaryOperator easing, double deadzone) {
        public static AxisSettings of(DoubleUnaryOperator easing, double deadzone) {
            return new AxisSettings(AGGRESSIVE, 0);
        }
    };

    public static final DoubleUnaryOperator LINEAR = in -> in;
    public static final DoubleUnaryOperator AGGRESSIVE = in -> ((2.0/3.0) * in) + ((1.0/3.0) * Math.pow(in, 3));
    public static final DoubleUnaryOperator INTERMEDIATE = in -> ((1.0/2.0) * in) + ((1.0/2.0) * Math.pow(in, 3));
    public static final DoubleUnaryOperator GENTLE = in -> ((1.0/3.0) * in) + ((2.0/3.0) * Math.pow(in, 3));
    public static final DoubleUnaryOperator CUBIC = in -> Math.pow(in, 3);

    public double getValue() {
        double in = controller.getRawAxis(controller.getVariant().getAxis(permanentId));

        double value = Math.abs(in) < deadzone ? 0 : in;

        return easing.applyAsDouble(value);
    }

    public BooleanSupplier axisHigherThan(double value) {
        return () -> this.getValue() > value;
    }
}