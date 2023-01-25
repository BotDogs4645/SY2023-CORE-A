package frc.bdlib.driver;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.bdlib.driver.JoystickAxisAIO.AxisSettings;

public class Driver {
    public static final double maxChassisSpeed = 3.96;
    public static final double maxRotationSpeed = 4; // radians per segund
    public static final AxisSettings normal = AxisSettings.of(JoystickAxisAIO.GENTLE, .15);

    public static final record DriverInfo(
        AxisSettings leftX, AxisSettings leftY, AxisSettings rightX, // Driver joystick axis configs
        double maxSpeed, double maxRotationSpeed, // multiply by %
        NeutralMode angleMode, // Angle brake or coast, brake will break stuff faster but can be more precise
        double rampTime // time in seconds to reach throttle position
    ) {};

    public static enum Profile {
        Average(new DriverInfo(
            normal, normal, normal,
            0.8 * maxRotationSpeed, 0.5 * maxChassisSpeed,
            NeutralMode.Coast,
            0
        )),

        LittleDrew(new DriverInfo(
            normal, normal, normal,
            0.8 * maxRotationSpeed, 0.5 * maxChassisSpeed,
            NeutralMode.Coast,
            0
        ))
        ;

        DriverInfo info;
        private Profile(DriverInfo info) {
            this.info = info;
        }

        public DriverInfo info() {
            return info;
        }
    }
}