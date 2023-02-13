package frc.bdlib.custom_talon;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import frc.bdlib.misc.BDManager;
import frc.bdlib.misc.BDUpdatable;

/**
* Custom implementation of the {@code WPI_TalonFX} class.
* @author David Muchow
* @version 1.0.0
*/
public class TalonFXW extends WPI_TalonFX implements BDUpdatable {
    // Units. Changes how the outputs of the motor are converted when we use get methods.
    private final SensorUnits sensor_units;
    private final FXWConfig configuration;
    private final int id;

    /**
     * Settings for Talons with gearing and attached to a wheel-like object. Base measurements should be in imperial.
     * @param gearing gear ratio of the connection to the
     * @param diameter diameter of the wheel-like object attached to the motor in inches.
     */
    public record FXWConfig(double gearing, double diameter) {

        public FXWConfig {
            diameter /= 12;
        }

        /**
         * Settings for Talons directly attached to a wheel-like object. Gearing is assumed to be one.
         * @param diameter diameter of the wheel-like object attached to the motor in inches.
        */
        public FXWConfig(double diameter) {
            this(diameter, 1.0);
        }
    }

    /**
     * For Talon controllers on the CANivore CAN bus with desired unit conversions.
     * @param can_id device ID of the Talon controller
     * @param can_bus id of the CAN bus, most likely "canivore" or "swerve"
     * @param configuration the {@link TalonFXW.FXWConfig} configuration that signifies the gearing ratio and attached object diameter of the motor.
     * @param sensor_units the decided units via the {@link SensorUnits} enum class.
     */
    public TalonFXW(int can_id, String can_bus, FXWConfig configuration, SensorUnits sensor_units) {
        super(can_id, can_bus);
        this.id = can_id;
        this.configuration = configuration;

        // Retain default value
        this.sensor_units = sensor_units == null ? SensorUnits.CTRE : sensor_units;

        //BDManager.getInstance().register(this);
    }

    /**
     * For Talon controllers on a CANivore.
     * @param can_id device ID of the Talon controller
     * @param can_bus id of the CAN bus, most likely "canivore" or "swerve"
     * @param configuration the {@link TalonFXW.FXWConfig} configuration that signifies the gearing ratio and attached object diameter of the motor.
     */
    public TalonFXW(int can_id, String can_bus, FXWConfig configuration) {
        this(can_id, can_bus, configuration, null);
    }

    /**
     * For Talon controllers on the roboRIO CAN bus with desired unit conversions.
     * @param can_id device ID of the Talon controller
     * @param configuration the {@link TalonFXW.FXWConfig} configuration that signifies the gearing ratio and attached object diameter of the motor.
     * @param sensor_units the decided units via the {@link SensorUnits} enum class.
     */
    public TalonFXW(int can_id, FXWConfig configuration, SensorUnits sensor_units) {
        this(can_id, "", configuration, sensor_units);
    }

    /**
     * For Talon controllers on the roboRIO CAN bus.
     * @param can_id device id of the Talon controller
     * @param configuration the {@link TalonFXW.FXWConfig} configuration that signifies the gearing ratio and attached object diameter of the motor.
    */
    public TalonFXW(int can_id, FXWConfig configuration) {
        this(can_id, configuration, null);
    }

    /**
     * @return How many rotations the shaft has made.
    */
    public double getShaftRotations() {
        return super.getSelectedSensorPosition() / 2048;
    }
    
    /**
     * @return How many full rotations the shaft makes every 100ms
    */
    public double getShaftVelocity() {
        return super.getSelectedSensorVelocity() / 2048;
    }

    /**
     * @return How many rotations the object, through gearing, makes per second.
    */
    public double getObjectRotationsPerSecond() {
        return (getShaftVelocity() * 10) / configuration.gearing();
    }

    /**
     * @return How many rotations the object, through gearing, makes per minute.
    */
    public double getObjectRotationsPerMinute() {
        return getObjectRotationsPerSecond() * 60;
    }

    public double getObjectTotalRotations() {
        return (getShaftRotations() / configuration.gearing());
    }

    /**
     * Depending on the sensor mode, you will either get:<p>
     * <b>Metric:</b> Total wheel rotations made in meters
     * <b>Imperial:</b> Total wheel rotations in feet
     * <b>CTRE:</b> Total wheel rotations in well.. rotations.
     * @return The total wheel rotations in selected unit.
     */
    public double getObjectTotalDistanceTraveled() {
        return switch (sensor_units) {
            case METRIC -> Units.feetToMeters(getObjectTotalRotations() * configuration.diameter() * Math.PI);
            case IMPERIAL -> getObjectTotalRotations() * configuration.diameter() * Math.PI;
            case CTRE -> getObjectTotalRotations();
        };
    }

    /**
     * Depending on the sensor mode, you will either get:<p>
     * <b>Metric:</b> Current wheel rotations in meters per second<p>
     * <b>Imperial:</b> Current wheel rotations in feet per second<p>
     * <b>CTRE:</b> Current wheel rotations in rotations per second.<p>
     * @return The wheel velocity in selected unit.
     */
    public double getObjectConvertedVelocity() {
        return switch (sensor_units) {
            case METRIC -> Units.feetToMeters(getObjectRotationsPerSecond() * configuration.diameter() * Math.PI);
            case IMPERIAL -> getObjectRotationsPerSecond() * configuration.diameter() * Math.PI;
            case CTRE -> getObjectRotationsPerSecond();
        };
    }

    /**
     * @return The configuration of this particular motor.
    */
    public FXWConfig getConfig() {
        return configuration;
    }

    /** <i>Warning:</i> might be error prone.
     * Sets the position of the motor to zero.
    */
    public void zero() {
        super.setSelectedSensorPosition(0, 0, 0);
    }

    public void update() {
        
    }

    public String getID() {
        return "TalonFXW " + id;
    }

    public String getStatus() {
        Faults current_faults = new Faults();
        super.getFaults(current_faults);

        return current_faults.hasAnyFault() ? current_faults.toString() : "OK";
    }
}