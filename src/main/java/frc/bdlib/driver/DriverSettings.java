package frc.bdlib.driver;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.bdlib.driver.JoystickAxisAIO.AxisSettings;

public class DriverSettings {
    public static final double maxChassisSpeed = 3.96;
    public static final double maxRotationSpeed = 4; // radians per segund

    public static final record Driver(
        AxisSettings leftX, AxisSettings leftY, AxisSettings rightX, // Driver joystick axis configs
        double maxSpeedPercentage, double maxRotationSpeedPercentage, // what % of max to use as their max
        NeutralMode angleMode, // Angle brake or coast, brake will break stuff faster but can be more precise
        double rampTime // time in seconds to reach throttle position
    ) {};
}
